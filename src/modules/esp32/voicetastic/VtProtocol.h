#pragma once

#include "configuration.h"

#if defined(ARCH_ESP32) && !MESHTASTIC_EXCLUDE_VOICETASTIC

#include <stdint.h>
#include <stddef.h>

namespace voicetastic {

// Wire-format constants from VOICE_PROTOCOL.md (v2).
static constexpr uint8_t  PROTOCOL_VERSION = 0x02;
static constexpr size_t   HEADER_SIZE      = 16;   // 12 logical bytes + 4-byte MAC trailer
static constexpr size_t   MAX_PACKET_SIZE  = 231;  // Meshtastic LoRa MTU
static constexpr size_t   MAX_BODY_SIZE    = MAX_PACKET_SIZE - HEADER_SIZE; // 215
static constexpr size_t   MAC_TAG_SIZE     = 4;
static constexpr uint8_t  MAX_CHUNKS_PER_MESSAGE = 255;
static constexpr uint8_t  MAX_PARITY_PER_MESSAGE = 128;

// type_flags bit layout (spec Appendix B).
static constexpr uint8_t  MASK_PACKET_TYPE   = 0xC0;  // bits 6-7
static constexpr uint8_t  MASK_ENCRYPTED     = 0x20;  // bit 5
static constexpr uint8_t  MASK_LAST_IN_STREAM = 0x10; // bit 4
static constexpr uint8_t  MASK_MAC_KEYED     = 0x08;  // bit 3
static constexpr uint8_t  MASK_RESERVED      = 0x07;  // bits 0-2 (must be zero)
static constexpr uint8_t  PACKET_TYPE_SHIFT  = 6;

enum class PacketType : uint8_t {
    DATA    = 0,
    PARITY  = 1,
    NACK    = 2,
    RESERVED = 3,  // receivers MUST drop
};

// Codec field (spec §3.2).
enum class CodecId : uint8_t {
    AMR_NB    = 0,
    OPUS      = 1,
    PCM_S16LE = 2,
    CODEC2    = 3,
};

// Codec2 mode ordinals (spec §3.2.2) -- map directly to the ESP32_Codec2 lib's
// CODEC2_MODE_* constants but the on-wire ordinal is what the spec defines.
enum class Codec2Mode : uint8_t {
    M_3200 = 0,
    M_2400 = 1,
    M_1600 = 2,
    M_1400 = 3,
    M_1300 = 4,
    M_1200 = 5,   // default for this implementation
};

struct VtHeader {
    uint8_t    version;        // 0x02
    PacketType packet_type;
    bool       encrypted;
    bool       last_in_stream;
    bool       mac_keyed;      // false for this build (we use SHA-256, not HMAC)
    uint32_t   message_id;
    CodecId    codec;
    uint8_t    codec_param;    // codec-specific ordinal
    uint8_t    stream_seq;
    uint8_t    chunk_index;
    uint8_t    total_data;
    uint8_t    parity_count;
};

// Encode the 16-byte header into `out`. Computes the MAC over the first 12
// bytes via SHA-256 (mac_keyed=false). Returns HEADER_SIZE on success or 0
// if `h` violates a spec rejection rule that we can catch pre-emission:
//   - h.version != 0x02
//   - h.packet_type == RESERVED
//   - h.total_data == 0 (except for NACK frames, which still echo a value)
//   - h.parity_count > MAX_PARITY_PER_MESSAGE
//   - mac_keyed == true (not supported by this build)
//   - reserved bits 0..2 implicitly zero
size_t encodeHeader(const VtHeader &h, uint8_t out[HEADER_SIZE]);

// Decode a 16-byte header. Returns true iff:
//   - in[0] == 0x02
//   - MAC tag matches SHA-256(in[0..12])[0..4]   (mac_keyed=0 path)
//   - packet_type != RESERVED
//   - mac_keyed == 0  (HMAC variant not supported yet)
//   - total_data != 0  (per spec §9.2 rejection rule)
//   - parity_count <= MAX_PARITY_PER_MESSAGE
// On false: `out` is left in an undefined state.
bool decodeHeader(const uint8_t in[HEADER_SIZE], VtHeader &out);

// Compute the 4-byte MAC tag for a given 12-byte logical header.
// Internal helper, exposed for test/debug use.
void computeHeaderMac(const uint8_t header12[12], uint8_t out_tag[MAC_TAG_SIZE]);

// Debug helper: dump a header in human-readable form to the log.
void logHeader(const char *prefix, const VtHeader &h);

} // namespace voicetastic

#endif // ARCH_ESP32 && !MESHTASTIC_EXCLUDE_VOICETASTIC
