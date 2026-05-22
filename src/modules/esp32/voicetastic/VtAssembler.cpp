#include "VtAssembler.h"

#if defined(ARCH_ESP32) && defined(HAS_VOICETASTIC) && !MESHTASTIC_EXCLUDE_VOICETASTIC

#include "VtInbox.h"
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
    // No free slot: evict the oldest in-progress (spec §9.1) and blacklist its
    // key so late shards from the evicted message don't immediately re-allocate.
    AssemblyState *oldest = &states_[0];
    for (auto &s : states_) {
        if ((int32_t)(s.started_ms - oldest->started_ms) < 0) oldest = &s;
    }
    addToBlacklist(oldest->from, oldest->message_id, millis());
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
    s.last_chunk_ms = 0;
    s.nack_rounds = 0;
    s.data_shards.clear();
    s.parity_shards.clear();
    s.data_received.clear();
    s.parity_received.clear();
    s.last_data_pending.clear();
    s.last_data_pending.shrink_to_fit();
}

size_t VtAssembler::inProgressCount() const
{
    size_t n = 0;
    for (const auto &s : states_) if (s.in_use) n++;
    return n;
}

bool VtAssembler::isBlacklisted(NodeNum from, uint32_t message_id, uint32_t now_ms)
{
    for (auto &e : blacklist_) {
        if (!e.in_use) continue;
        // Lazy expiry: drop on lookup once the TTL elapses, so we don't have to
        // walk the ring on every tick. `expires_ms - now_ms` wraps cleanly as a
        // signed delta even across the 49-day millis() wraparound.
        if ((int32_t)(e.expires_ms - now_ms) <= 0) {
            e.in_use = false;
            continue;
        }
        if (e.from == from && e.message_id == message_id) return true;
    }
    return false;
}

void VtAssembler::addToBlacklist(NodeNum from, uint32_t message_id, uint32_t now_ms)
{
    // Don't double-insert: if it's already there, refresh the expiry instead.
    for (auto &e : blacklist_) {
        if (e.in_use && e.from == from && e.message_id == message_id) {
            e.expires_ms = now_ms + BLACKLIST_TTL_MS;
            return;
        }
    }
    // Prefer an expired/unused slot before overwriting fresh entries.
    for (auto &e : blacklist_) {
        if (!e.in_use || (int32_t)(e.expires_ms - now_ms) <= 0) {
            e.in_use = true;
            e.from = from;
            e.message_id = message_id;
            e.expires_ms = now_ms + BLACKLIST_TTL_MS;
            return;
        }
    }
    // All slots fresh: round-robin overwrite at blacklist_next.
    BlacklistEntry &e = blacklist_[blacklist_next];
    blacklist_next = (blacklist_next + 1) % BLACKLIST_MAX;
    e.in_use = true;
    e.from = from;
    e.message_id = message_id;
    e.expires_ms = now_ms + BLACKLIST_TTL_MS;
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

    const uint32_t now_ms = millis();
    if (isBlacklisted(from, h.message_id, now_ms)) {
        // Spec §9.1: already-finalized message; ignore late drain shards.
        return false;
    }

    AssemblyState *s = findExisting(from, h.message_id);
    if (s == nullptr) {
        s = allocateSlot(from, h.message_id);
        s->started_ms = now_ms;
        s->last_chunk_ms = now_ms;
        s->nack_rounds = 0;
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
    // Any accepted frame extends the quiet window — NACK timer restarts.
    s->last_chunk_ms = now_ms;

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
            // If a lone final-DATA arrived before chunk_size was known, its
            // body was stashed in last_data_pending. Fold it into the final
            // data shard now (spec §4). Drop the stash if it doesn't fit the
            // newly-inferred chunk size — that means the sender used a
            // chunk_size smaller than the lone-final body, which is illegal
            // and the chunk would have failed the same-size check below.
            if (s->last_data_seen && !s->last_data_pending.empty() &&
                s->last_data_pending.size() <= (size_t)s->chunk_size) {
                const uint8_t fi = (uint8_t)(s->total_data - 1);
                std::vector<uint8_t> &dst = s->data_shards[fi];
                dst.assign(s->chunk_size, 0);
                memcpy(dst.data(), s->last_data_pending.data(), s->last_data_pending.size());
                if (!s->data_received[fi]) {
                    s->data_received[fi] = true;
                    s->received_data_count++;
                }
                s->last_data_pending.clear();
                s->last_data_pending.shrink_to_fit();
            } else if (!s->last_data_pending.empty()) {
                // Stash was for a bogus chunk_size — discard it.
                LOG_WARN("vtAssembler: mid=%08x dropping stashed final-DATA (%u > chunk_size %u)",
                         (unsigned)h.message_id,
                         (unsigned)s->last_data_pending.size(),
                         (unsigned)s->chunk_size);
                s->last_data_pending.clear();
                s->last_data_pending.shrink_to_fit();
            }
        } else {
            // Lone final-DATA frame arrived first; defer chunk_size discovery
            // (spec §4). Stash the body verbatim in the sidecar so we can fold
            // it into data_shards once chunk_size becomes known. A retransmit
            // of the same final-DATA with the same length is fine — it just
            // overwrites identical bytes. A retransmit with a different length
            // is a sender bug or tampering; reject without disturbing the
            // already-stashed copy.
            if (body_len > MAX_BODY_SIZE) return false;
            if (s->last_data_seen) {
                if (body_len != s->last_data_real_size) {
                    LOG_WARN("vtAssembler: mid=%08x stashed final-DATA len drift %u vs %u — drop",
                             (unsigned)h.message_id, (unsigned)body_len,
                             (unsigned)s->last_data_real_size);
                    return false;
                }
            } else {
                s->last_data_real_size = body_len;
            }
            s->last_data_seen = true;
            s->last_data_pending.assign(body, body + body_len);
            return false;
        }
    }

    // Body length validation now that chunk_size is known.
    if (h.packet_type == PacketType::DATA) {
        if (is_final_data) {
            if (body_len > s->chunk_size) return false;
            // Final-DATA retransmits MUST carry the same trimmed length as
            // the first copy we accepted. A sender bug (or attacker tampering)
            // delivering a different length would otherwise rewrite the shard
            // bytes against a different boundary while we keep the original
            // last_data_real_size — finalize would then emit audio that's
            // length-consistent with the header but byte-corrupt at the tail.
            // Drop the frame instead.
            if (s->last_data_seen) {
                if (body_len != s->last_data_real_size) {
                    LOG_WARN("vtAssembler: mid=%08x final-DATA len drift %u vs %u — drop",
                             (unsigned)h.message_id, (unsigned)body_len,
                             (unsigned)s->last_data_real_size);
                    return false;
                }
            } else {
                s->last_data_real_size = body_len;
            }
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

    // Build the rs:: shard arrays. Both arrays are bounded by the protocol
    // caps (255 data + 128 parity = 383 entries), so they live on the stack —
    // keeps this RX hot path off the heap and removes a silent OOM crash mode
    // versus the previous `new bool[total_shards]` (no null-check).
    // std::vector<bool> is a bit-packed specialisation (no .data()), so we use
    // a plain bool[] for `present_buf`.
    static constexpr int kMaxShards =
        (int)MAX_CHUNKS_PER_MESSAGE + (int)MAX_PARITY_PER_MESSAGE; // 255 + 128
    uint8_t *ptrs[kMaxShards];
    bool     present_buf[kMaxShards];
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
                                    (size_t)s.chunk_size, ptrs, present_buf);
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
    msg.audio_size = total_len;

    // Prefer SD storage when available. Saves ~5 KB / message of PSRAM and
    // keeps the pending_play_queue's permanent footprint to ~kMaxInbox metadata
    // structs (~120 B each) instead of audio buffers. RAM fallback runs when
    // no SD is mounted, or if the write fails partway.
    bool stored_on_sd = false;
    if (inbox::available()) {
        const size_t wrote = inbox::store(msg.message_id, s.data_shards,
                                          s.total_data, s.chunk_size, final_size);
        if (wrote == total_len) {
            stored_on_sd = true;
            // msg.audio stays empty — playback worker streams from SD.
        } else {
            LOG_WARN("vtAssembler: inbox::store failed (wrote=%u expected=%u), keeping in RAM",
                     (unsigned)wrote, (unsigned)total_len);
        }
    }
    if (!stored_on_sd) {
        msg.audio.reserve(total_len);
        for (uint8_t i = 0; i < s.total_data; i++) {
            const std::vector<uint8_t> &shard = s.data_shards[i];
            const size_t take = (i == s.total_data - 1) ? final_size : s.chunk_size;
            msg.audio.insert(msg.audio.end(), shard.begin(), shard.begin() + take);
        }
    }
    msg.recovered_via_fec = (uint8_t)(s.total_data - msg.received_data);

    LOG_INFO("vtAssembler: mid=%08x complete (%u bytes %s, %u/%u direct + %u FEC)",
             (unsigned)msg.message_id, (unsigned)msg.audio_size,
             stored_on_sd ? "on SD" : "in RAM",
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

    // Spec §9.1: remember this (from, message_id) so late shards from the
    // sender's drain queue don't restart a phantom partial reassembly.
    addToBlacklist(s.from, s.message_id, millis());

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
            // Spec §9.1: a partial timeout is still a finalization for blacklist
            // purposes — late shards must not resurrect this assembly.
            addToBlacklist(s.from, s.message_id, now_ms);
            resetSlot(s);
        }
    }
}

bool VtAssembler::pollPendingNack(PendingNack &out, uint32_t now_ms)
{
    for (auto &s : states_) {
        if (!s.in_use) continue;
        // Spec §3.4 / Reliability-FEC-and-NACK.md: receivers MUST NOT emit
        // NACKs for broadcast messages — every listener would NACK the same
        // chunks and flood the sender, which has no way to pick a retransmit
        // target anyway. Broadcasts rely on FEC + partial-on-timeout; the
        // assembly's `tick()` still drives the absolute timeout.
        if (s.to == NODENUM_BROADCAST) continue;
        // Need to know total_data and have at least one missing chunk to NACK.
        if (s.received_data_count == s.total_data) continue;
        if ((now_ms - s.last_chunk_ms) < NACK_QUIET_MS) continue;

        const bool exhausted = (s.nack_rounds >= NACK_MAX_ROUNDS);

        out.from         = s.from;
        out.to           = s.to;
        out.channel      = s.channel;
        out.message_id   = s.message_id;
        out.codec        = s.codec;
        out.codec_param  = s.codec_param;
        out.stream_seq   = s.stream_seq;
        out.total_data   = s.total_data;
        out.parity_count = s.parity_count;
        out.give_up      = exhausted;
        out.missing.assign(s.total_data, false);
        for (uint8_t i = 0; i < s.total_data; ++i) {
            out.missing[i] = !s.data_received[i];
        }

        if (exhausted) {
            LOG_WARN("vtAssembler: mid=%08x NACK rounds exhausted, giving up at %u/%u",
                     (unsigned)s.message_id, (unsigned)s.received_data_count,
                     (unsigned)s.total_data);
            // Finalize partial: blacklist + drop state. We still emit one final
            // give_up NACK so the sender can stop draining its queue.
            addToBlacklist(s.from, s.message_id, now_ms);
            resetSlot(s);
        } else {
            s.nack_rounds++;
            // Restart the quiet timer so we don't immediately re-NACK on the
            // next tick before the sender has a chance to respond.
            s.last_chunk_ms = now_ms;
        }
        return true;
    }
    return false;
}

} // namespace voicetastic

#endif // ARCH_ESP32 && !MESHTASTIC_EXCLUDE_VOICETASTIC
