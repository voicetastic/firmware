#pragma once

#include "configuration.h"

#if defined(ARCH_ESP32) && defined(HAS_VOICETASTIC) && !MESHTASTIC_EXCLUDE_VOICETASTIC

#include "VtProtocol.h"
#include "mesh/generated/meshtastic/config.pb.h"
#include <stdint.h>
#include <stddef.h>
#include <vector>

namespace voicetastic {

// Chunk size per modem preset (spec §4). Returns 96 for unknown presets.
uint8_t chunkSizeForPreset(meshtastic_Config_LoRaConfig_ModemPreset preset);

// Pacing delay between chunks per modem preset (spec §2.1). Returns 500 ms
// for unknown presets (spec default).
uint32_t pacingMsForPreset(meshtastic_Config_LoRaConfig_ModemPreset preset);

// Maximum audio bytes that fit in a single message at a given chunk_size
// (spec §4: chunk_size × 255).
inline size_t maxAudioBytes(uint8_t chunk_size) { return (size_t)chunk_size * MAX_CHUNKS_PER_MESSAGE; }

// Default parity count for `total_data` original chunks. ~20% (spec §5
// "Medium/mixed"), capped at MAX_PARITY_PER_MESSAGE, minimum 1.
uint8_t defaultParityCount(uint8_t total_data);

// A message ready to send: shards padded to chunk_size, plus the real length
// of the final data chunk (so the sender can emit it trimmed per spec §4).
struct OutboundMessage {
    uint32_t   message_id;
    uint8_t    stream_seq;
    CodecId    codec;
    uint8_t    codec_param;     // codec-specific (e.g. Codec2Mode ordinal)
    uint8_t    chunk_size;
    uint8_t    total_data;
    uint8_t    parity_count;
    size_t     last_data_real_size; // <= chunk_size; trimmed wire length of last DATA frame
    std::vector<std::vector<uint8_t>> data;    // total_data entries, each chunk_size bytes (last is zero-padded for FEC)
    std::vector<std::vector<uint8_t>> parity;  // parity_count entries, each chunk_size bytes
};

// Build an OutboundMessage from a raw encoded-audio buffer.
//
//   audio:        codec-encoded bytes (e.g. Codec2 frames concatenated)
//   audio_len:    bytes in `audio`. Must be > 0 and <= maxAudioBytes(chunkSizeForPreset(preset)).
//   preset:       used to pick chunk_size and default parity_count.
//   codec, codec_param: copied into the result.
//   message_id, stream_seq: caller-chosen identifiers.
//
// Returns true on success. Computes parity shards via rs::encode(). The
// final data shard is internally zero-padded to chunk_size (needed for FEC);
// `last_data_real_size` records the un-padded length for use on the wire.
bool buildOutbound(const uint8_t *audio, size_t audio_len,
                   meshtastic_Config_LoRaConfig_ModemPreset preset,
                   CodecId codec, uint8_t codec_param,
                   uint32_t message_id, uint8_t stream_seq,
                   OutboundMessage &out);

} // namespace voicetastic

#endif // ARCH_ESP32 && !MESHTASTIC_EXCLUDE_VOICETASTIC
