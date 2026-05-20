#include "VtProtocol.h"

#if defined(ARCH_ESP32) && !MESHTASTIC_EXCLUDE_VOICETASTIC

#include "CryptoEngine.h"
#include "configuration.h"
#include "mbedtls/gcm.h"
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

// HMAC-SHA256 over a single contiguous input. Local helper since ESP-IDF's
// mbedtls is built without `mbedtls_hkdf` exposed; we synthesize HKDF below.
static bool hmacSha256Single(const uint8_t *key, size_t key_len,
                             const uint8_t *msg, size_t msg_len,
                             uint8_t out[32])
{
    const mbedtls_md_info_t *info = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
    if (!info) return false;
    return mbedtls_md_hmac(info, key, key_len, msg, msg_len, out) == 0;
}

bool deriveEnvelopeKey(const uint8_t *psk, size_t psk_len,
                       uint32_t message_id, uint32_t from_node_num,
                       uint8_t out_key[GCM_KEY_LEN])
{
    if (psk == nullptr || psk_len == 0) return false;

    uint8_t ikm[8];
    put_be32(ikm + 0, message_id);
    put_be32(ikm + 4, from_node_num);

    // HKDF-SHA256 (RFC 5869). L = 32 ≤ HashLen, so the expand step needs a
    // single block:
    //   PRK = HMAC-SHA256(salt = psk, ikm)
    //   T(1) = HMAC-SHA256(PRK, info ‖ 0x01)
    //   OKM = T(1)[..32]
    // Spec §7: the HKDF `info` string is permanent across protocol revisions
    // ("voicetastic/v2"), not bumped with the wire version byte.
    static const uint8_t kInfo[] = "voicetastic/v2";
    constexpr size_t kInfoLen = sizeof(kInfo) - 1;

    uint8_t prk[32];
    if (!hmacSha256Single(psk, psk_len, ikm, sizeof(ikm), prk)) return false;

    uint8_t expand_in[kInfoLen + 1];
    memcpy(expand_in, kInfo, kInfoLen);
    expand_in[kInfoLen] = 0x01;
    if (!hmacSha256Single(prk, sizeof(prk), expand_in, sizeof(expand_in), out_key)) return false;
    return true;
}

bool gcmEncrypt(const uint8_t key[GCM_KEY_LEN], const uint8_t nonce[GCM_NONCE_LEN],
                const uint8_t *aad, size_t aad_len,
                const uint8_t *plain, size_t plain_len,
                uint8_t *ct_out, uint8_t tag_out[GCM_TAG_LEN])
{
    mbedtls_gcm_context ctx;
    mbedtls_gcm_init(&ctx);
    bool ok = false;
    if (mbedtls_gcm_setkey(&ctx, MBEDTLS_CIPHER_ID_AES, key, (unsigned)(GCM_KEY_LEN * 8)) == 0) {
        ok = mbedtls_gcm_crypt_and_tag(&ctx, MBEDTLS_GCM_ENCRYPT, plain_len,
                                       nonce, GCM_NONCE_LEN,
                                       aad, aad_len,
                                       plain, ct_out,
                                       GCM_TAG_LEN, tag_out) == 0;
    }
    mbedtls_gcm_free(&ctx);
    return ok;
}

bool gcmDecrypt(const uint8_t key[GCM_KEY_LEN], const uint8_t nonce[GCM_NONCE_LEN],
                const uint8_t *aad, size_t aad_len,
                const uint8_t *ct, size_t ct_len,
                const uint8_t tag[GCM_TAG_LEN],
                uint8_t *pt_out)
{
    mbedtls_gcm_context ctx;
    mbedtls_gcm_init(&ctx);
    bool ok = false;
    if (mbedtls_gcm_setkey(&ctx, MBEDTLS_CIPHER_ID_AES, key, (unsigned)(GCM_KEY_LEN * 8)) == 0) {
        ok = mbedtls_gcm_auth_decrypt(&ctx, ct_len,
                                      nonce, GCM_NONCE_LEN,
                                      aad, aad_len,
                                      tag, GCM_TAG_LEN,
                                      ct, pt_out) == 0;
    }
    mbedtls_gcm_free(&ctx);
    return ok;
}

size_t encodeNackBody(uint8_t total_data, const bool *missing, bool give_up,
                      uint8_t *out, size_t out_max)
{
    if (total_data == 0 || missing == nullptr || out == nullptr) return 0;
    const size_t bitmap_len = ((size_t)total_data + 7u) / 8u;
    const size_t needed = 2 + bitmap_len;
    if (out_max < needed) return 0;

    out[0] = NACK_VERSION;
    out[1] = give_up ? NACK_FLAG_GIVE_UP : 0;
    // MSB-first per byte: bit 0 of byte 2 = chunk 0.
    memset(out + 2, 0, bitmap_len);
    for (uint8_t i = 0; i < total_data; ++i) {
        if (missing[i]) {
            const size_t byte_off = 2 + (i / 8);
            const uint8_t bit = (uint8_t)(0x80u >> (i % 8));
            out[byte_off] |= bit;
        }
    }
    return needed;
}

bool decodeNackBody(const uint8_t *body, size_t body_len, uint8_t total_data,
                    bool *missing_out, bool &give_up_out)
{
    if (body == nullptr || missing_out == nullptr || total_data == 0) return false;
    const size_t bitmap_len = ((size_t)total_data + 7u) / 8u;
    if (body_len < 2 + bitmap_len) return false;
    if (body[0] != NACK_VERSION) return false;
    if ((body[1] & ~NACK_FLAG_GIVE_UP) != 0) return false; // reserved bits

    give_up_out = (body[1] & NACK_FLAG_GIVE_UP) != 0;
    for (uint8_t i = 0; i < total_data; ++i) {
        const size_t byte_off = 2 + (i / 8);
        const uint8_t bit = (uint8_t)(0x80u >> (i % 8));
        missing_out[i] = (body[byte_off] & bit) != 0;
    }
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
