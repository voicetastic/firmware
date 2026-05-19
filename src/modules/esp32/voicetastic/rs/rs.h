#pragma once

#include "configuration.h"

#if defined(ARCH_ESP32) && !MESHTASTIC_EXCLUDE_VOICETASTIC

#include <stdint.h>
#include <stddef.h>

// Shard-level Reed-Solomon over GF(2^8), wire-compatible with the
// `reed-solomon-erasure` Rust crate / klauspost/reedsolomon (Go), which the
// Voicetastic v2 spec uses (VOICE_PROTOCOL.md §5).
//
// GF(2^8) with irreducible polynomial 0x11d (x^8 + x^4 + x^3 + x^2 + 1),
// generator alpha = 2. Vandermonde matrix V[i][j] = i^j (with 0^0 = 1).
// Encoding matrix M = V * inverse(top data-rows of V), making M's top
// data×data block the identity (systematic) and the bottom parity×data block
// the parity coefficients.
//
// Notation: a "shard" is `shard_size` bytes. There are `data_shards` original
// shards plus `parity_shards` redundant shards. Any `data_shards` shards out
// of the total (in any combination of data and parity) are sufficient to
// recover the original data.

namespace voicetastic { namespace rs {

static constexpr int MAX_SHARDS = 255;            // 8-bit indexing in GF(2^8)
static constexpr int MAX_DATA_SHARDS = 255;
static constexpr int MAX_PARITY_SHARDS = 128;     // matches spec §11 limit

// Idempotent. Builds GF(2^8) log/exp tables on first call.
void init();

// Encode parity shards from data shards.
//
//   data_shards:    1..MAX_DATA_SHARDS
//   parity_shards:  0..MAX_PARITY_SHARDS, with data+parity <= MAX_SHARDS
//   shard_size:     bytes per shard (must be > 0)
//   data:           array of `data_shards` pointers, each to `shard_size` bytes
//   parity_out:     array of `parity_shards` pointers, each to a pre-allocated
//                   `shard_size`-byte output buffer (will be overwritten)
//
// Returns true on success, false on invalid arguments.
bool encode(int data_shards, int parity_shards, size_t shard_size,
            const uint8_t *const *data, uint8_t *const *parity_out);

// Reconstruct missing shards.
//
//   data_shards, parity_shards, shard_size: same as encode().
//   shards:    array of (data+parity) pointers; each present shard points to
//              valid `shard_size` bytes. Each missing shard's slot must point
//              to a pre-allocated `shard_size`-byte buffer that will be filled
//              with the reconstructed contents.
//   present:   array of (data+parity) booleans. true = shards[i] is valid input.
//
// The implementation reconstructs only what's needed to recover the *data*
// shards; missing parity shards (if any) remain in whatever state the caller
// left them.
//
// Returns true on success. Returns false if fewer than `data_shards` shards
// are present (unrecoverable) or arguments are invalid.
bool decode(int data_shards, int parity_shards, size_t shard_size,
            uint8_t *const *shards, const bool *present);

// Boot-time self-test: encode known data, drop some shards, decode, compare.
// Logs results via LOG_INFO / LOG_ERROR. Cheap; call once at module init.
void runSelfTest();

}} // namespace voicetastic::rs

#endif // ARCH_ESP32 && !MESHTASTIC_EXCLUDE_VOICETASTIC
