#include "VtChunker.h"

#if defined(ARCH_ESP32) && !MESHTASTIC_EXCLUDE_VOICETASTIC

#include "rs/rs.h"
#include "configuration.h"
#include <string.h>

namespace voicetastic {

uint8_t chunkSizeForPreset(meshtastic_Config_LoRaConfig_ModemPreset preset)
{
    using P = meshtastic_Config_LoRaConfig_ModemPreset;
    switch (preset) {
    case P::meshtastic_Config_LoRaConfig_ModemPreset_SHORT_TURBO:
    case P::meshtastic_Config_LoRaConfig_ModemPreset_SHORT_FAST:
    case P::meshtastic_Config_LoRaConfig_ModemPreset_SHORT_SLOW:
        return 219;
    case P::meshtastic_Config_LoRaConfig_ModemPreset_MEDIUM_FAST:
    case P::meshtastic_Config_LoRaConfig_ModemPreset_MEDIUM_SLOW:
        return 160;
    case P::meshtastic_Config_LoRaConfig_ModemPreset_LONG_FAST:
    case P::meshtastic_Config_LoRaConfig_ModemPreset_LONG_MODERATE:
    case P::meshtastic_Config_LoRaConfig_ModemPreset_LONG_SLOW:
        return 96;
    case P::meshtastic_Config_LoRaConfig_ModemPreset_VERY_LONG_SLOW:
        return 48;
    default:
        return 96; // unknown / conservative default
    }
}

uint32_t pacingMsForPreset(meshtastic_Config_LoRaConfig_ModemPreset preset)
{
    using P = meshtastic_Config_LoRaConfig_ModemPreset;
    switch (preset) {
    case P::meshtastic_Config_LoRaConfig_ModemPreset_SHORT_TURBO:
    case P::meshtastic_Config_LoRaConfig_ModemPreset_SHORT_FAST:
        return 100;
    case P::meshtastic_Config_LoRaConfig_ModemPreset_SHORT_SLOW:
    case P::meshtastic_Config_LoRaConfig_ModemPreset_MEDIUM_FAST:
        return 200;
    case P::meshtastic_Config_LoRaConfig_ModemPreset_MEDIUM_SLOW:
    case P::meshtastic_Config_LoRaConfig_ModemPreset_LONG_FAST:
        return 350;
    case P::meshtastic_Config_LoRaConfig_ModemPreset_LONG_MODERATE:
    case P::meshtastic_Config_LoRaConfig_ModemPreset_LONG_SLOW:
        return 500;
    case P::meshtastic_Config_LoRaConfig_ModemPreset_VERY_LONG_SLOW:
        return 800;
    default:
        return 500; // spec §2.1: unknown preset MUST default to 500 ms
    }
}

uint8_t defaultParityCount(uint8_t total_data)
{
    uint16_t p = (uint16_t)total_data / 5;   // ~20%
    if (p < 1) p = 1;
    if (p > MAX_PARITY_PER_MESSAGE) p = MAX_PARITY_PER_MESSAGE;
    if ((int)total_data + (int)p > 255) p = (uint16_t)(255 - total_data);
    return (uint8_t)p;
}

bool buildOutbound(const uint8_t *audio, size_t audio_len,
                   meshtastic_Config_LoRaConfig_ModemPreset preset,
                   CodecId codec, uint8_t codec_param,
                   uint32_t message_id, uint8_t stream_seq,
                   OutboundMessage &out)
{
    if (audio == nullptr || audio_len == 0) return false;

    const uint8_t chunk_size = chunkSizeForPreset(preset);
    if (chunk_size == 0) return false;
    if (audio_len > maxAudioBytes(chunk_size)) return false;

    const size_t total_data_sz = (audio_len + chunk_size - 1) / chunk_size;
    if (total_data_sz == 0 || total_data_sz > MAX_CHUNKS_PER_MESSAGE) return false;
    const uint8_t total_data = (uint8_t)total_data_sz;
    const uint8_t parity_count = defaultParityCount(total_data);

    out.message_id  = message_id;
    out.stream_seq  = stream_seq;
    out.codec       = codec;
    out.codec_param = codec_param;
    out.chunk_size  = chunk_size;
    out.total_data  = total_data;
    out.parity_count = parity_count;

    out.data.assign(total_data, std::vector<uint8_t>(chunk_size, 0));
    out.parity.assign(parity_count, std::vector<uint8_t>(chunk_size, 0));

    // Split & zero-pad.
    for (uint8_t i = 0; i < total_data; i++) {
        const size_t off = (size_t)i * chunk_size;
        const size_t take = (audio_len - off) >= chunk_size ? (size_t)chunk_size : (audio_len - off);
        memcpy(out.data[i].data(), audio + off, take);
        // Trailing bytes are already 0 from the assign() initializer.
        if (i == total_data - 1) {
            out.last_data_real_size = take;
        }
    }

    // FEC: produce parity shards.
    if (parity_count > 0) {
        std::vector<const uint8_t *> dptrs((size_t)total_data);
        std::vector<uint8_t *>       pptrs((size_t)parity_count);
        for (uint8_t i = 0; i < total_data; i++)  dptrs[i] = out.data[i].data();
        for (uint8_t i = 0; i < parity_count; i++) pptrs[i] = out.parity[i].data();
        if (!rs::encode((int)total_data, (int)parity_count, (size_t)chunk_size,
                        dptrs.data(), pptrs.data())) {
            return false;
        }
    }
    return true;
}

} // namespace voicetastic

#endif // ARCH_ESP32 && !MESHTASTIC_EXCLUDE_VOICETASTIC
