#include "VoicetasticModule.h"

#if defined(ARCH_ESP32) && !MESHTASTIC_EXCLUDE_VOICETASTIC

#include "MeshService.h"
#include "NodeDB.h"
#include "Router.h"
#include "VtProtocol.h"

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
    // No periodic work yet. Re-check in 1 s once we have a TX queue or
    // assembler timeouts to tick.
    return 1000;
}

#endif // ARCH_ESP32 && !MESHTASTIC_EXCLUDE_VOICETASTIC
