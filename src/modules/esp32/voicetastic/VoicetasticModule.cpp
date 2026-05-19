#include "VoicetasticModule.h"

#if defined(ARCH_ESP32) && !MESHTASTIC_EXCLUDE_VOICETASTIC

#include "MeshService.h"
#include "NodeDB.h"
#include "RadioInterface.h"
#include "Router.h"
#include "VtProtocol.h"
#include "rs/rs.h"
#include <Arduino.h>
#include <esp_random.h>
#include <string.h>

VoicetasticModule *voicetasticModule;

// Round-trip a known header through the codec at boot so any wire-format
// regression shows up immediately in the serial log. Cheap; runs once.
static void runProtocolSelfTest()
{
    using namespace voicetastic;
    VtHeader h{};
    h.version        = PROTOCOL_VERSION;
    h.packet_type    = PacketType::DATA;
    h.encrypted      = false;
    h.last_in_stream = false;
    h.mac_keyed      = false;
    h.message_id     = 0xDEADBEEF;
    h.codec          = CodecId::CODEC2;
    h.codec_param    = (uint8_t)Codec2Mode::M_1200;
    h.stream_seq     = 7;
    h.chunk_index    = 3;
    h.total_data     = 22;
    h.parity_count   = 5;

    uint8_t buf[HEADER_SIZE] = {0};
    if (encodeHeader(h, buf) != HEADER_SIZE) {
        LOG_ERROR("Voicetastic selftest: encodeHeader failed");
        return;
    }

    VtHeader r{};
    if (!decodeHeader(buf, r)) {
        LOG_ERROR("Voicetastic selftest: decodeHeader rejected our own output");
        return;
    }

    const bool ok =
        r.version == h.version && r.packet_type == h.packet_type && r.encrypted == h.encrypted &&
        r.last_in_stream == h.last_in_stream && r.message_id == h.message_id && r.codec == h.codec &&
        r.codec_param == h.codec_param && r.stream_seq == h.stream_seq && r.chunk_index == h.chunk_index &&
        r.total_data == h.total_data && r.parity_count == h.parity_count;

    if (ok) {
        LOG_INFO("Voicetastic selftest: header codec OK (mid=%08x)", (unsigned)r.message_id);
    } else {
        LOG_ERROR("Voicetastic selftest: round-trip mismatch");
    }
}

VoicetasticModule::VoicetasticModule()
    : SinglePortModule("Voicetastic", meshtastic_PortNum_PRIVATE_APP), concurrency::OSThread("Voicetastic")
{
    LOG_INFO("Voicetastic module init (port=%d, scope=TX+RX Codec2 plaintext+FEC)", (int)meshtastic_PortNum_PRIVATE_APP);
    runProtocolSelfTest();
    voicetastic::rs::runSelfTest();
    boot_ms = millis();
}

bool VoicetasticModule::enqueueOutbound(const uint8_t *audio, size_t audio_len,
                                        voicetastic::CodecId codec, uint8_t codec_param,
                                        NodeNum to)
{
    using namespace voicetastic;
    if (tx_active) {
        LOG_WARN("Voicetastic: TX busy, drop enqueue");
        return false;
    }
    const auto preset = (meshtastic_Config_LoRaConfig_ModemPreset)config.lora.modem_preset;
    const uint32_t mid = esp_random() | 1u; // non-zero u32 per spec §6
    const uint8_t  seq = stream_seq_counter++;

    if (!buildOutbound(audio, audio_len, preset, codec, codec_param, mid, seq, tx_msg)) {
        LOG_ERROR("Voicetastic: buildOutbound failed (audio_len=%u preset=%d)",
                  (unsigned)audio_len, (int)preset);
        return false;
    }

    tx_to = to;
    tx_next_chunk = 0;
    tx_paced_until_ms = 0;
    tx_active = true;
    LOG_INFO("Voicetastic: enqueue mid=%08x total_data=%u parity=%u chunk=%u last_real=%u",
             (unsigned)tx_msg.message_id, (unsigned)tx_msg.total_data,
             (unsigned)tx_msg.parity_count, (unsigned)tx_msg.chunk_size,
             (unsigned)tx_msg.last_data_real_size);
    return true;
}

void VoicetasticModule::sendOneChunk()
{
    using namespace voicetastic;
    const uint16_t idx = tx_next_chunk;
    const bool is_data = idx < tx_msg.total_data;
    const uint16_t parity_idx = is_data ? 0 : (idx - tx_msg.total_data);
    const bool is_last_in_stream =
        (uint16_t)(idx + 1) == (uint16_t)tx_msg.total_data + (uint16_t)tx_msg.parity_count;

    VtHeader h{};
    h.version        = PROTOCOL_VERSION;
    h.packet_type    = is_data ? PacketType::DATA : PacketType::PARITY;
    h.encrypted      = false;
    h.last_in_stream = is_last_in_stream;
    h.mac_keyed      = false;
    h.message_id     = tx_msg.message_id;
    h.codec          = tx_msg.codec;
    h.codec_param    = tx_msg.codec_param;
    h.stream_seq     = tx_msg.stream_seq;
    h.chunk_index    = is_data ? (uint8_t)idx : (uint8_t)parity_idx;
    h.total_data     = tx_msg.total_data;
    h.parity_count   = tx_msg.parity_count;

    uint8_t header_buf[HEADER_SIZE] = {0};
    if (encodeHeader(h, header_buf) != HEADER_SIZE) {
        LOG_ERROR("Voicetastic: header encode failed (idx=%u)", (unsigned)idx);
        return;
    }

    // Determine body size: trimmed for final DATA, full for all others.
    const std::vector<uint8_t> &src = is_data ? tx_msg.data[idx] : tx_msg.parity[parity_idx];
    const size_t body_len =
        (is_data && idx == tx_msg.total_data - 1) ? tx_msg.last_data_real_size : (size_t)tx_msg.chunk_size;

    meshtastic_MeshPacket *p = allocDataPacket();
    p->to = tx_to;
    p->want_ack = false; // no NACK loop in this build
    p->decoded.want_response = false;
    p->priority = meshtastic_MeshPacket_Priority_DEFAULT;

    const size_t total = HEADER_SIZE + body_len;
    if (total > sizeof(p->decoded.payload.bytes)) {
        LOG_ERROR("Voicetastic: chunk too large (%u)", (unsigned)total);
        // Allocated packet must be freed; in this codebase service->sendToMesh
        // takes ownership only on send. Fall through with size=0 to avoid leak.
        p->decoded.payload.size = 0;
    } else {
        memcpy(p->decoded.payload.bytes, header_buf, HEADER_SIZE);
        memcpy(p->decoded.payload.bytes + HEADER_SIZE, src.data(), body_len);
        p->decoded.payload.size = total;
    }

    LOG_DEBUG("Voicetastic: tx mid=%08x %s ci=%u/%u body=%u",
              (unsigned)h.message_id, is_data ? "D" : "P",
              (unsigned)h.chunk_index, (unsigned)tx_msg.total_data, (unsigned)body_len);
    service->sendToMesh(p);
}

ProcessMessage VoicetasticModule::handleReceived(const meshtastic_MeshPacket &mp)
{
    using namespace voicetastic;
    const auto &p = mp.decoded;

    if (p.payload.size < HEADER_SIZE) {
        LOG_DEBUG("Voicetastic: drop short PRIVATE_APP packet (%u bytes)", (unsigned)p.payload.size);
        return ProcessMessage::CONTINUE;
    }

    VtHeader h{};
    if (!decodeHeader(p.payload.bytes, h)) {
        LOG_DEBUG("Voicetastic: header reject (ver=0x%02x size=%u)", (unsigned)p.payload.bytes[0],
                  (unsigned)p.payload.size);
        return ProcessMessage::CONTINUE;
    }

    const size_t body_len = p.payload.size - HEADER_SIZE;
    char prefix[40];
    snprintf(prefix, sizeof(prefix), "rx from=0x%08x body=%u", (unsigned)mp.from, (unsigned)body_len);
    logHeader(prefix, h);

    // Phase 5 will route DATA/PARITY frames to the assembler. For now: log only.
    return ProcessMessage::CONTINUE;
}

int32_t VoicetasticModule::runOnce()
{
    using namespace voicetastic;
    const uint32_t now = millis();

    // One-shot boot test broadcast: gives an end-to-end interop check against
    // voicetastic-desktop receivers without needing a mic. Will be removed in
    // Phase 4 in favour of a mic-triggered path.
    if (!boot_test_sent && (now - boot_ms) > 10000) {
        boot_test_sent = true;
        static const char kTestPayload[] = "Voicetastic firmware test broadcast - phase 3b";
        enqueueOutbound((const uint8_t *)kTestPayload, sizeof(kTestPayload) - 1,
                        CodecId::CODEC2, (uint8_t)Codec2Mode::M_1200, NODENUM_BROADCAST);
    }

    if (!tx_active) return 500;

    // Done?
    const uint16_t total_chunks = (uint16_t)tx_msg.total_data + (uint16_t)tx_msg.parity_count;
    if (tx_next_chunk >= total_chunks) {
        LOG_INFO("Voicetastic: TX complete mid=%08x (%u chunks)",
                 (unsigned)tx_msg.message_id, (unsigned)total_chunks);
        tx_active = false;
        // Release the shard buffers so we don't hold ~tens of KB indefinitely.
        tx_msg.data.clear(); tx_msg.parity.clear();
        return 500;
    }

    // Pacing window not yet elapsed?
    if ((int32_t)(now - tx_paced_until_ms) < 0) {
        return (int32_t)(tx_paced_until_ms - now);
    }

    // Firmware-queue backpressure (spec §2.2): pause while < 3 free slots.
    meshtastic_QueueStatus qs = router->getQueueStatus();
    if (qs.free <= 2) {
        return 100;
    }

    sendOneChunk();
    tx_next_chunk++;
    const auto preset = (meshtastic_Config_LoRaConfig_ModemPreset)config.lora.modem_preset;
    tx_paced_until_ms = now + pacingMsForPreset(preset);

    return 10; // re-enter quickly; pacing/backpressure gates above
}

#endif // ARCH_ESP32 && !MESHTASTIC_EXCLUDE_VOICETASTIC
