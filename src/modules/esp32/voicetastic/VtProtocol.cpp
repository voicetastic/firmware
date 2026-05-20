#include "VtProtocol.h"

#if defined(ARCH_ESP32) && !MESHTASTIC_EXCLUDE_VOICETASTIC

#include "CryptoEngine.h"
#include "configuration.h"
#include "mbedtls/md.h"
#include <string.h>

namespace voicetastic {

static inline void put_be32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v >> 24);
    p[1] = (uint8_t)(v >> 16);
    p[2] = (uint8_t)(v >> 8);
    p[3] = (uint8_t)v;
}

static inline uint32_t get_be32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | (uint32_t)p[3];
}

// Constant-time 4-byte compare. Avoids leaking a partial-match oracle on the
// keyed path; 32 bits is short enough that a naive `memcmp` would not be
// realistically exploitable but the discipline is cheap.
static inline bool macEqual(const uint8_t *a, const uint8_t *b)
{
    uint8_t diff = 0;
    for (size_t i = 0; i < MAC_TAG_SIZE; ++i) diff |= (uint8_t)(a[i] ^ b[i]);
    return diff == 0;
}

void computeHeaderMac(const uint8_t header12[12], uint8_t out_tag[MAC_TAG_SIZE],
                      const uint8_t *mac_key, size_t mac_key_len)
{
    if (mac_key && mac_key_len > 0) {
        // HMAC-SHA256(mac_key, header12) truncated to 4 bytes.
        uint8_t digest[32];
        const mbedtls_md_info_t *info = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
        if (info && mbedtls_md_hmac(info, mac_key, mac_key_len, header12, 12, digest) == 0) {
            memcpy(out_tag, digest, MAC_TAG_SIZE);
            return;
        }
        // mbedtls failure path: fall through to unkeyed so we still emit *something*
        // deterministic rather than uninitialized bytes. The receiver will then
        // reject the frame on MAC mismatch, which is the safe failure mode.
    }
    // Plain SHA-256(header12) truncated to 4 bytes. Reuses CryptoEngine's
    // in-place hash; we work in a scratch buffer big enough for the digest.
    uint8_t scratch[32] = {0};
    memcpy(scratch, header12, 12);
    crypto->hash(scratch, 12);
    memcpy(out_tag, scratch, MAC_TAG_SIZE);
}

size_t encodeHeader(VtHeader &h, uint8_t out[HEADER_SIZE],
                    const uint8_t *mac_key, size_t mac_key_len)
{
    if (h.version != PROTOCOL_VERSION)
        return 0;
    if (h.packet_type == PacketType::RESERVED)
        return 0;
    if (h.packet_type != PacketType::NACK && h.total_data == 0)
        return 0;
    if (h.parity_count > MAX_PARITY_PER_MESSAGE)
        return 0;

    // Force the mac_keyed flag to match whether we actually have a key. The
    // flag bit is part of the bytes the MAC covers, so it MUST be set before
    // computing the tag.
    h.mac_keyed = (mac_key != nullptr && mac_key_len > 0);

    uint8_t type_flags = 0;
    type_flags |= (uint8_t)((uint8_t)h.packet_type << PACKET_TYPE_SHIFT) & MASK_PACKET_TYPE;
    if (h.encrypted)        type_flags |= MASK_ENCRYPTED;
    if (h.last_in_stream)   type_flags |= MASK_LAST_IN_STREAM;
    if (h.mac_keyed)        type_flags |= MASK_MAC_KEYED;
    // Reserved bits implicitly 0.

    out[0]  = h.version;
    out[1]  = type_flags;
    put_be32(out + 2, h.message_id);
    out[6]  = (uint8_t)h.codec;
    out[7]  = h.codec_param;
    out[8]  = h.stream_seq;
    out[9]  = h.chunk_index;
    out[10] = h.total_data;
    out[11] = h.parity_count;

    computeHeaderMac(out, out + 12, mac_key, mac_key_len);
    return HEADER_SIZE;
}

bool decodeHeader(const uint8_t in[HEADER_SIZE], VtHeader &out,
                  const uint8_t *mac_key, size_t mac_key_len)
{
    if (in[0] != PROTOCOL_VERSION)
        return false;

    const uint8_t type_flags = in[1];
    if ((type_flags & MASK_RESERVED) != 0)
        return false; // spec §3: bits 0-2 reserved, must be zero
    const PacketType pt = (PacketType)((type_flags & MASK_PACKET_TYPE) >> PACKET_TYPE_SHIFT);
    if (pt == PacketType::RESERVED)
        return false;

    const bool keyed = (type_flags & MASK_MAC_KEYED) != 0;
    if (keyed && (!mac_key || mac_key_len == 0))
        return false; // header advertises a keyed MAC but we have no PSK to verify it

    // Verify MAC against header[0..12], using the same primitive the sender
    // advertised. macEqual is constant-time; mismatch is the by-far dominant
    // rejection path on a noisy channel.
    uint8_t expected[MAC_TAG_SIZE];
    if (keyed) {
        computeHeaderMac(in, expected, mac_key, mac_key_len);
    } else {
        computeHeaderMac(in, expected, nullptr, 0);
    }
    if (!macEqual(expected, in + 12))
        return false;

    const uint8_t total_data = in[10];
    const uint8_t parity_count = in[11];
    if (total_data == 0)
        return false;
    if (parity_count > MAX_PARITY_PER_MESSAGE)
        return false;

    out.version        = in[0];
    out.packet_type    = pt;
    out.encrypted      = (type_flags & MASK_ENCRYPTED) != 0;
    out.last_in_stream = (type_flags & MASK_LAST_IN_STREAM) != 0;
    out.mac_keyed      = keyed;
    out.message_id     = get_be32(in + 2);
    out.codec          = (CodecId)in[6];
    out.codec_param    = in[7];
    out.stream_seq     = in[8];
    out.chunk_index    = in[9];
    out.total_data     = total_data;
    out.parity_count   = parity_count;
    return true;
}

void logHeader(const char *prefix, const VtHeader &h)
{
    const char *type_str =
        h.packet_type == PacketType::DATA     ? "DATA"     :
        h.packet_type == PacketType::PARITY   ? "PARITY"   :
        h.packet_type == PacketType::NACK     ? "NACK"     : "RES";
    LOG_INFO("%s vt2 mid=%08x %s ci=%u/%u par=%u codec=%u/%u seq=%u%s%s",
             prefix,
             (unsigned)h.message_id, type_str,
             (unsigned)h.chunk_index, (unsigned)h.total_data, (unsigned)h.parity_count,
             (unsigned)h.codec, (unsigned)h.codec_param,
             (unsigned)h.stream_seq,
             h.encrypted ? " enc" : "",
             h.last_in_stream ? " last" : "");
}

} // namespace voicetastic

#endif // ARCH_ESP32 && !MESHTASTIC_EXCLUDE_VOICETASTIC
