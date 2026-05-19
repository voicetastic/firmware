#include "VoicetasticModule.h"

#if defined(ARCH_ESP32) && !MESHTASTIC_EXCLUDE_VOICETASTIC

#include "MeshService.h"
#include "NodeDB.h"
#include "Router.h"

VoicetasticModule *voicetasticModule;

VoicetasticModule::VoicetasticModule()
    : SinglePortModule("Voicetastic", meshtastic_PortNum_PRIVATE_APP), concurrency::OSThread("Voicetastic")
{
    LOG_INFO("Voicetastic module init (port=%d, scope=TX+RX Codec2 plaintext+FEC)", (int)meshtastic_PortNum_PRIVATE_APP);
}

ProcessMessage VoicetasticModule::handleReceived(const meshtastic_MeshPacket &mp)
{
    const auto &p = mp.decoded;

    if (p.payload.size < 16) {
        LOG_DEBUG("Voicetastic: drop short PRIVATE_APP packet (%u bytes)", (unsigned)p.payload.size);
        return ProcessMessage::CONTINUE;
    }

    const uint8_t version = p.payload.bytes[0];
    if (version != 0x02) {
        LOG_DEBUG("Voicetastic: drop unknown protocol version 0x%02x", version);
        return ProcessMessage::CONTINUE;
    }

    LOG_INFO("Voicetastic: rx %u bytes from 0x%08x (v2)", (unsigned)p.payload.size, (unsigned)mp.from);

    // Phase 2+ will parse and route to the assembler. For now: log only.
    return ProcessMessage::CONTINUE;
}

int32_t VoicetasticModule::runOnce()
{
    // No periodic work yet. Re-check in 1 s once we have a TX queue or
    // assembler timeouts to tick.
    return 1000;
}

#endif // ARCH_ESP32 && !MESHTASTIC_EXCLUDE_VOICETASTIC
