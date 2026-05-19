#include "VtAssembler.h"

#if defined(ARCH_ESP32) && !MESHTASTIC_EXCLUDE_VOICETASTIC

#include "configuration.h"
#include "rs/rs.h"
#include <string.h>

namespace voicetastic {

VtAssembler::AssemblyState *VtAssembler::findExisting(NodeNum from, uint32_t message_id)
{
    for (auto &s : states_) {
        if (s.in_use && s.from == from && s.message_id == message_id) return &s;
    }
    return nullptr;
}

VtAssembler::AssemblyState *VtAssembler::allocateSlot(NodeNum from, uint32_t message_id)
{
    // Prefer the first unused slot.
    for (auto &s : states_) {
        if (!s.in_use) {
            resetSlot(s);
            s.in_use = true;
            s.from = from;
            s.message_id = message_id;
            return &s;
        }
    }
    // No free slot: evict the oldest in-progress (per spec §9.1 eviction policy,
    // simplified — we don't blacklist).
    AssemblyState *oldest = &states_[0];
    for (auto &s : states_) {
        if ((int32_t)(s.started_ms - oldest->started_ms) < 0) oldest = &s;
    }
    resetSlot(*oldest);
    oldest->in_use = true;
    oldest->from = from;
    oldest->message_id = message_id;
    return oldest;
}

void VtAssembler::resetSlot(AssemblyState &s)
{
    s.in_use = false;
    s.chunk_size_known = false;
    s.last_data_real_size = 0;
    s.last_data_seen = false;
    s.received_data_count = 0;
    s.data_shards.clear();
    s.parity_shards.clear();
    s.data_received.clear();
    s.parity_received.clear();
}

size_t VtAssembler::inProgressCount() const
{
    size_t n = 0;
    for (const auto &s : states_) if (s.in_use) n++;
    return n;
}

bool VtAssembler::acceptFrame(NodeNum from, NodeNum to, uint8_t channel,
                              const VtHeader &h, const uint8_t *body, size_t body_len)
{
    using namespace voicetastic;
    // Per spec §9.2 rejection rules (subset we can check at this layer):
    if (h.packet_type != PacketType::DATA && h.packet_type != PacketType::PARITY) {
        // NACKs and reserved frames are ignored in this build.
        return false;
    }
    if (h.total_data == 0) return false;
    if (h.parity_count > MAX_PARITY_PER_MESSAGE) return false;
    if (h.packet_type == PacketType::DATA && h.chunk_index >= h.total_data) return false;
    if (h.packet_type == PacketType::PARITY && h.chunk_index >= h.parity_count) return false;
    if (body_len == 0 || body_len > MAX_BODY_SIZE) return false;

    AssemblyState *s = findExisting(from, h.message_id);
    if (s == nullptr) {
        s = allocateSlot(from, h.message_id);
        s->started_ms = millis();
        s->to = to;
        s->channel = channel;
        s->codec = h.codec;
        s->codec_param = h.codec_param;
        s->stream_seq = h.stream_seq;
        s->total_data = h.total_data;
        s->parity_count = h.parity_count;
        s->data_received.assign(h.total_data, false);
        s->parity_received.assign(h.parity_count, false);
        s->data_shards.assign(h.total_data, std::vector<uint8_t>());
        s->parity_shards.assign(h.parity_count, std::vector<uint8_t>());
        LOG_INFO("vtAssembler: new mid=%08x from=0x%08x total=%u parity=%u",
                 (unsigned)h.message_id, (unsigned)from,
                 (unsigned)h.total_data, (unsigned)h.parity_count);
    } else {
        // Existing assembly — sanity check immutable header fields.
        if (s->codec != h.codec || s->codec_param != h.codec_param ||
            s->total_data != h.total_data) {
            // Drift; drop the frame.
            return false;
        }
        // parity_count MAY grow but not shrink (spec §5). We don't currently
        // handle late retransmits adding new parity rows, so just reject shrinks.
        if (h.parity_count < s->parity_count) return false;
    }

    // Infer chunk_size from the first PARITY frame or any non-final DATA frame
    // whose body length is unambiguous (spec §4).
    const bool is_final_data = (h.packet_type == PacketType::DATA &&
                                h.chunk_index == h.total_data - 1);
    if (!s->chunk_size_known) {
        if (h.packet_type == PacketType::PARITY || !is_final_data) {
            s->chunk_size = (uint8_t)body_len;
            s->chunk_size_known = true;
            // Resize per-shard buffers now that we know the chunk size.
            for (auto &v : s->data_shards)   v.assign(s->chunk_size, 0);
            for (auto &v : s->parity_shards) v.assign(s->chunk_size, 0);
            LOG_DEBUG("vtAssembler: mid=%08x chunk_size inferred = %u",
                      (unsigned)h.message_id, (unsigned)s->chunk_size);
        } else {
            // Lone final-DATA frame arrived first; defer chunk_size discovery
            // (spec §4). Keep the frame around in a sidecar.
            s->last_data_real_size = body_len;
            s->last_data_seen = true;
            // We can't size the shard yet — record will be filled in later when
            // chunk_size becomes known.
            return false;
        }
    }

    // Body length validation now that chunk_size is known.
    if (h.packet_type == PacketType::DATA) {
        if (is_final_data) {
            if (body_len > s->chunk_size) return false;
            if (!s->last_data_seen) s->last_data_real_size = body_len;
            s->last_data_seen = true;
        } else {
            if (body_len != s->chunk_size) return false;
        }
        if (!s->data_received[h.chunk_index]) {
            // Zero-pad the last DATA chunk into the full-size shard for FEC math.
            std::vector<uint8_t> &dst = s->data_shards[h.chunk_index];
            dst.assign(s->chunk_size, 0);
            memcpy(dst.data(), body, body_len);
            s->data_received[h.chunk_index] = true;
            s->received_data_count++;
        }
    } else { // PARITY
        if (body_len != s->chunk_size) return false;
        if (!s->parity_received[h.chunk_index]) {
            std::vector<uint8_t> &dst = s->parity_shards[h.chunk_index];
            dst.assign(s->chunk_size, 0);
            memcpy(dst.data(), body, body_len);
            s->parity_received[h.chunk_index] = true;
        }
    }

    return tryFinalize(*s);
}

bool VtAssembler::tryFinalize(AssemblyState &s)
{
    using namespace voicetastic;

    // Fast path: every data shard arrived directly.
    if (s.received_data_count == s.total_data) {
        publish(s);
        return true;
    }

    // We need the final DATA's real length before we can finalize via FEC
    // (spec §5 final-chunk caveat).
    if (!s.last_data_seen) return false;

    // Count total received shards (data + parity).
    size_t present_total = s.received_data_count;
    for (bool p : s.parity_received) if (p) present_total++;
    if (present_total < s.total_data) return false; // not enough yet

    // Build the rs:: shard arrays. std::vector<bool> is a bit-packed
    // specialisation (no .data()), so use a plain bool[] here.
    const int total_shards = s.total_data + s.parity_count;
    std::vector<uint8_t *> ptrs((size_t)total_shards);
    bool *present_buf = new bool[total_shards];
    for (int i = 0; i < (int)s.total_data; i++) {
        if (!s.data_received[i]) {
            // Placeholder; rs::decode will overwrite missing data shards.
            s.data_shards[i].assign(s.chunk_size, 0);
        }
        ptrs[i] = s.data_shards[i].data();
        present_buf[i] = s.data_received[i];
    }
    for (int i = 0; i < (int)s.parity_count; i++) {
        ptrs[(int)s.total_data + i] = s.parity_shards[i].data();
        present_buf[(int)s.total_data + i] = s.parity_received[i];
    }

    const bool decoded = rs::decode((int)s.total_data, (int)s.parity_count,
                                    (size_t)s.chunk_size, ptrs.data(), present_buf);
    delete[] present_buf;
    if (!decoded) return false; // not actually enough shards in usable combination

    // Mark all data shards as present after successful FEC.
    for (int i = 0; i < (int)s.total_data; i++) s.data_received[i] = true;

    publish(s);
    return true;
}

void VtAssembler::publish(AssemblyState &s)
{
    using namespace voicetastic;

    // Assemble the codec stream: all DATA shards back-to-back; the final shard
    // is trimmed to last_data_real_size.
    ReceivedVoiceMessage msg{};
    msg.from = s.from;
    msg.to = s.to;
    msg.channel = s.channel;
    msg.message_id = s.message_id;
    msg.codec = s.codec;
    msg.codec_param = s.codec_param;
    msg.total_data = s.total_data;
    msg.received_data = s.received_data_count;   // before FEC fill
    msg.completed_ms = millis();

    const size_t final_size = s.last_data_real_size ? s.last_data_real_size : s.chunk_size;
    const size_t total_len = (size_t)(s.total_data - 1) * s.chunk_size + final_size;
    msg.audio.reserve(total_len);
    for (uint8_t i = 0; i < s.total_data; i++) {
        const std::vector<uint8_t> &shard = s.data_shards[i];
        const size_t take = (i == s.total_data - 1) ? final_size : s.chunk_size;
        msg.audio.insert(msg.audio.end(), shard.begin(), shard.begin() + take);
    }
    msg.recovered_via_fec = (uint8_t)(s.total_data - msg.received_data);

    LOG_INFO("vtAssembler: mid=%08x complete (%u bytes, %u/%u direct + %u FEC)",
             (unsigned)msg.message_id, (unsigned)msg.audio.size(),
             (unsigned)msg.received_data, (unsigned)msg.total_data,
             (unsigned)msg.recovered_via_fec);

    // Drop the oldest if the ring is full.
    if (complete_count >= MAX_COMPLETE_QUEUE) {
        complete_head = (complete_head + 1) % MAX_COMPLETE_QUEUE;
        complete_count--;
    }
    complete_[complete_tail] = std::move(msg);
    complete_tail = (complete_tail + 1) % MAX_COMPLETE_QUEUE;
    complete_count++;

    resetSlot(s);
}

bool VtAssembler::popComplete(ReceivedVoiceMessage &out)
{
    if (complete_count == 0) return false;
    out = std::move(complete_[complete_head]);
    complete_head = (complete_head + 1) % MAX_COMPLETE_QUEUE;
    complete_count--;
    return true;
}

void VtAssembler::tick(uint32_t now_ms)
{
    for (auto &s : states_) {
        if (!s.in_use) continue;
        if ((now_ms - s.started_ms) > TIMEOUT_MS) {
            LOG_WARN("vtAssembler: mid=%08x timed out at %u/%u shards",
                     (unsigned)s.message_id, (unsigned)s.received_data_count,
                     (unsigned)s.total_data);
            resetSlot(s);
        }
    }
}

} // namespace voicetastic

#endif // ARCH_ESP32 && !MESHTASTIC_EXCLUDE_VOICETASTIC
