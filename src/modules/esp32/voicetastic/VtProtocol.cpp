#include "VtProtocol.h"

#if defined(ARCH_ESP32) && !MESHTASTIC_EXCLUDE_VOICETASTIC

#include "CryptoEngine.h"
#include "configuration.h"
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

void computeHeaderMac(const uint8_t header12[12], uint8_t out_tag[MAC_TAG_SIZE])
{
    // CryptoEngine::hash() hashes in-place: input bytes are overwritten by the
    // 32-byte SHA-256 digest. We work in a scratch buffer big enough for the
    // digest, copy the input, hash, then truncate.
    uint8_t scratch[32] = {0};
    memcpy(scratch, header12, 12);
    crypto->hash(scratch, 12);
    memcpy(out_tag, scratch, MAC_TAG_SIZE);
}

size_t encodeHeader(const VtHeader &h, uint8_t out[HEADER_SIZE])
{
    if (h.version != PROTOCOL_VERSION)
        return 0;
    if (h.packet_type == PacketType::RESERVED)
        return 0;
    if (h.mac_keyed)
        return 0; // HMAC-SHA256 variant not implemented in this build
    if (h.packet_type != PacketType::NACK && h.total_data == 0)
        return 0;
    if (h.parity_count > MAX_PARITY_PER_MESSAGE)
        return 0;

    uint8_t type_flags = 0;
    type_flags |= (uint8_t)((uint8_t)h.packet_type << PACKET_TYPE_SHIFT) & MASK_PACKET_TYPE;
    if (h.encrypted)        type_flags |= MASK_ENCRYPTED;
    if (h.last_in_stream)   type_flags |= MASK_LAST_IN_STREAM;
    // mac_keyed forced 0 (checked above); reserved bits implicitly 0.

    out[0]  = h.version;
    out[1]  = type_flags;
    put_be32(out + 2, h.message_id);
    out[6]  = (uint8_t)h.codec;
    out[7]  = h.codec_param;
    out[8]  = h.stream_seq;
    out[9]  = h.chunk_index;
    out[10] = h.total_data;
    out[11] = h.parity_count;

    computeHeaderMac(out, out + 12);
    return HEADER_SIZE;
}

bool decodeHeader(const uint8_t in[HEADER_SIZE], VtHeader &out)
{
    if (in[0] != PROTOCOL_VERSION)
        return false;

    const uint8_t type_flags = in[1];
    const PacketType pt = (PacketType)((type_flags & MASK_PACKET_TYPE) >> PACKET_TYPE_SHIFT);
    if (pt == PacketType::RESERVED)
        return false;

    const bool mac_keyed = (type_flags & MASK_MAC_KEYED) != 0;
    if (mac_keyed)
        return false; // not supported yet

    // Verify MAC against header[0..12].
    uint8_t expected[MAC_TAG_SIZE];
    computeHeaderMac(in, expected);
    if (memcmp(expected, in + 12, MAC_TAG_SIZE) != 0)
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
    out.mac_keyed      = false;
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
