#include "rs.h"

#if defined(ARCH_ESP32) && !MESHTASTIC_EXCLUDE_VOICETASTIC

#include "configuration.h"
#include <string.h>
#include <stdlib.h>

namespace voicetastic { namespace rs {

// --------- GF(2^8) with primitive polynomial 0x11d, generator alpha = 2 ----------
// Matches the convention used by klauspost/reedsolomon and reed-solomon-erasure.

static uint8_t s_exp[512];   // exp[i] = alpha^i, doubled to avoid mod in mul
static uint8_t s_log[256];   // log[exp[i]] = i (for i in 0..254); log[0] is unused
static bool    s_inited = false;

void init()
{
    if (s_inited) return;

    uint8_t x = 1;
    for (int i = 0; i < 255; i++) {
        s_exp[i] = x;
        s_log[x] = (uint8_t)i;
        // x = x * 2 in GF(2^8) with poly 0x11d
        uint16_t y = (uint16_t)x << 1;
        if (y & 0x100) y ^= 0x11d;
        x = (uint8_t)y;
    }
    // s_exp[255] == s_exp[0] == 1 (cycle); extend table to 510 for fast mul.
    for (int i = 255; i < 510; i++) s_exp[i] = s_exp[i - 255];
    s_log[0] = 0; // sentinel; do not use

    s_inited = true;
}

static inline uint8_t gmul(uint8_t a, uint8_t b)
{
    if (a == 0 || b == 0) return 0;
    return s_exp[(int)s_log[a] + (int)s_log[b]];
}

static inline uint8_t gdiv(uint8_t a, uint8_t b)
{
    // b must be nonzero
    if (a == 0) return 0;
    int d = (int)s_log[a] - (int)s_log[b];
    if (d < 0) d += 255;
    return s_exp[d];
}

// a^n in GF(2^8). Convention: 0^0 = 1, 0^n = 0 for n > 0 (matches klauspost).
static uint8_t gpow(uint8_t a, int n)
{
    if (n == 0) return 1;
    if (a == 0) return 0;
    int idx = ((int)s_log[a] * n) % 255;
    if (idx < 0) idx += 255;
    return s_exp[idx];
}

// --------- Matrix helpers (row-major, byte cells over GF(2^8)) ----------

// rows × cols matrix; entry (r, c) at m[r * cols + c].
static void vandermonde(int rows, int cols, uint8_t *m)
{
    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < cols; c++) {
            m[r * cols + c] = gpow((uint8_t)r, c);
        }
    }
}

// In-place invert an n×n matrix. Uses an n×(2n) augmented buffer. Returns true
// on success (matrix non-singular). `m` is overwritten with the inverse.
// `scratch` must hold n * 2n bytes.
static bool matrix_invert(uint8_t *m, int n, uint8_t *scratch)
{
    const int aug_cols = 2 * n;
    // Build [m | I] in scratch.
    for (int r = 0; r < n; r++) {
        memcpy(scratch + r * aug_cols, m + r * n, n);
        memset(scratch + r * aug_cols + n, 0, n);
        scratch[r * aug_cols + n + r] = 1;
    }

    for (int c = 0; c < n; c++) {
        // Find pivot.
        if (scratch[c * aug_cols + c] == 0) {
            int swap = -1;
            for (int r = c + 1; r < n; r++) {
                if (scratch[r * aug_cols + c] != 0) { swap = r; break; }
            }
            if (swap < 0) return false; // singular
            for (int k = 0; k < aug_cols; k++) {
                uint8_t t = scratch[c * aug_cols + k];
                scratch[c * aug_cols + k] = scratch[swap * aug_cols + k];
                scratch[swap * aug_cols + k] = t;
            }
        }
        // Scale pivot row to make scratch[c][c] == 1.
        uint8_t piv = scratch[c * aug_cols + c];
        if (piv != 1) {
            for (int k = 0; k < aug_cols; k++) {
                scratch[c * aug_cols + k] = gdiv(scratch[c * aug_cols + k], piv);
            }
        }
        // Eliminate column c from all other rows.
        for (int r = 0; r < n; r++) {
            if (r == c) continue;
            uint8_t f = scratch[r * aug_cols + c];
            if (f == 0) continue;
            for (int k = 0; k < aug_cols; k++) {
                scratch[r * aug_cols + k] ^= gmul(f, scratch[c * aug_cols + k]);
            }
        }
    }

    // Copy out the right half (inverse).
    for (int r = 0; r < n; r++) {
        memcpy(m + r * n, scratch + r * aug_cols + n, n);
    }
    return true;
}

// C = A × B over GF(2^8). A is a×b, B is b×c, C is a×c.
static void matrix_mul(const uint8_t *A, const uint8_t *B,
                       int a, int b, int c, uint8_t *C)
{
    for (int i = 0; i < a; i++) {
        for (int j = 0; j < c; j++) {
            uint8_t sum = 0;
            for (int k = 0; k < b; k++) {
                sum ^= gmul(A[i * b + k], B[k * c + j]);
            }
            C[i * c + j] = sum;
        }
    }
}

// Build the (data+parity) × data encoding matrix. Top `data` rows are the
// identity; bottom `parity` rows are the parity coefficients.
// `out` must be (data+parity) × data bytes.
static bool build_encoding_matrix(int data, int parity, uint8_t *out)
{
    const int total = data + parity;

    // V = Vandermonde, total × data.
    uint8_t *V = (uint8_t *)malloc((size_t)total * data);
    if (!V) return false;
    vandermonde(total, data, V);

    // Take top data × data submatrix and invert it.
    uint8_t *top = (uint8_t *)malloc((size_t)data * data);
    if (!top) { free(V); return false; }
    memcpy(top, V, (size_t)data * data);

    uint8_t *scratch = (uint8_t *)malloc((size_t)data * 2 * data);
    if (!scratch) { free(V); free(top); return false; }

    if (!matrix_invert(top, data, scratch)) {
        free(V); free(top); free(scratch);
        return false;
    }

    // M = V * inverse(top). Result is total × data, with top data rows = I.
    matrix_mul(V, top, total, data, data, out);

    free(V); free(top); free(scratch);
    return true;
}

// --------- Public API ----------

bool encode(int data_shards, int parity_shards, size_t shard_size,
            const uint8_t *const *data, uint8_t *const *parity_out)
{
    if (!s_inited) init();
    if (data_shards < 1 || data_shards > MAX_DATA_SHARDS) return false;
    if (parity_shards < 0 || parity_shards > MAX_PARITY_SHARDS) return false;
    if (data_shards + parity_shards > MAX_SHARDS) return false;
    if (shard_size == 0) return false;
    if (parity_shards == 0) return true; // nothing to do

    const int total = data_shards + parity_shards;
    uint8_t *M = (uint8_t *)malloc((size_t)total * data_shards);
    if (!M) return false;
    if (!build_encoding_matrix(data_shards, parity_shards, M)) { free(M); return false; }

    // Parity coefficient submatrix starts at row `data_shards`.
    const uint8_t *Mp = M + (size_t)data_shards * data_shards;

    for (int r = 0; r < parity_shards; r++) {
        uint8_t *out = parity_out[r];
        memset(out, 0, shard_size);
        for (int d = 0; d < data_shards; d++) {
            const uint8_t coef = Mp[r * data_shards + d];
            if (coef == 0) continue;
            const uint8_t *in = data[d];
            if (coef == 1) {
                for (size_t i = 0; i < shard_size; i++) out[i] ^= in[i];
            } else {
                for (size_t i = 0; i < shard_size; i++) out[i] ^= gmul(coef, in[i]);
            }
        }
    }

    free(M);
    return true;
}

bool decode(int data_shards, int parity_shards, size_t shard_size,
            uint8_t *const *shards, const bool *present)
{
    if (!s_inited) init();
    if (data_shards < 1 || data_shards > MAX_DATA_SHARDS) return false;
    if (parity_shards < 0 || parity_shards > MAX_PARITY_SHARDS) return false;
    const int total = data_shards + parity_shards;
    if (total > MAX_SHARDS) return false;
    if (shard_size == 0) return false;

    // Count present shards; collect indices.
    int n_present = 0;
    for (int i = 0; i < total; i++) if (present[i]) n_present++;
    if (n_present < data_shards) return false;

    // Quick path: if all data shards are present, nothing to reconstruct.
    bool all_data_present = true;
    for (int i = 0; i < data_shards; i++) if (!present[i]) { all_data_present = false; break; }
    if (all_data_present) return true;

    // Build the encoding matrix.
    uint8_t *M = (uint8_t *)malloc((size_t)total * data_shards);
    if (!M) return false;
    if (!build_encoding_matrix(data_shards, parity_shards, M)) { free(M); return false; }

    // Pick the first `data_shards` present shards as our "input" rows.
    // Build a data_shards × data_shards submatrix from M's rows, plus the
    // corresponding shard pointers.
    int chosen[MAX_DATA_SHARDS];
    uint8_t *sub = (uint8_t *)malloc((size_t)data_shards * data_shards);
    if (!sub) { free(M); return false; }
    int picked = 0;
    for (int i = 0; i < total && picked < data_shards; i++) {
        if (!present[i]) continue;
        memcpy(sub + (size_t)picked * data_shards, M + (size_t)i * data_shards, data_shards);
        chosen[picked++] = i;
    }

    // Invert the submatrix.
    uint8_t *scratch = (uint8_t *)malloc((size_t)data_shards * 2 * data_shards);
    if (!scratch) { free(M); free(sub); return false; }
    if (!matrix_invert(sub, data_shards, scratch)) {
        // Should not happen with Vandermonde construction unless data_shards
        // is too large for GF(2^8) — guarded by MAX_DATA_SHARDS.
        free(M); free(sub); free(scratch);
        return false;
    }
    free(scratch);

    // For each missing data shard i, its row in `sub_inverse` (== sub now)
    // tells us how to reconstruct it from the chosen shards.
    // Reconstructed[i] = XOR over k of (sub[i][k] * shards[chosen[k]]).
    for (int i = 0; i < data_shards; i++) {
        if (present[i]) continue;
        uint8_t *out = shards[i];
        memset(out, 0, shard_size);
        for (int k = 0; k < data_shards; k++) {
            uint8_t coef = sub[i * data_shards + k];
            if (coef == 0) continue;
            const uint8_t *in = shards[chosen[k]];
            if (coef == 1) {
                for (size_t b = 0; b < shard_size; b++) out[b] ^= in[b];
            } else {
                for (size_t b = 0; b < shard_size; b++) out[b] ^= gmul(coef, in[b]);
            }
        }
    }

    free(M);
    free(sub);
    return true;
}

void runSelfTest()
{
    init();

    // 4 data shards × 16 bytes each, plus 2 parity shards.
    constexpr int D = 4;
    constexpr int P = 2;
    constexpr size_t SZ = 16;

    uint8_t data[D][SZ];
    for (int r = 0; r < D; r++)
        for (size_t c = 0; c < SZ; c++)
            data[r][c] = (uint8_t)(r * 17 + c * 3 + 1);

    uint8_t parity[P][SZ];
    const uint8_t *dptrs[D] = {data[0], data[1], data[2], data[3]};
    uint8_t *pptrs[P]      = {parity[0], parity[1]};
    if (!encode(D, P, SZ, dptrs, pptrs)) {
        LOG_ERROR("Voicetastic RS selftest: encode failed");
        return;
    }

    // Drop data[1] and data[3]; reconstruct from data[0], data[2], parity[0], parity[1].
    uint8_t recov1[SZ] = {0}, recov3[SZ] = {0};
    uint8_t *all[D + P] = {data[0], recov1, data[2], recov3, parity[0], parity[1]};
    bool present[D + P] = {true, false, true, false, true, true};
    if (!decode(D, P, SZ, all, present)) {
        LOG_ERROR("Voicetastic RS selftest: decode failed");
        return;
    }

    if (memcmp(recov1, data[1], SZ) != 0 || memcmp(recov3, data[3], SZ) != 0) {
        LOG_ERROR("Voicetastic RS selftest: reconstruction mismatch");
        return;
    }

    LOG_INFO("Voicetastic RS selftest: OK (recovered 2 of %d data shards from %d+%d)", D, D, P);
}

}} // namespace voicetastic::rs

#endif // ARCH_ESP32 && !MESHTASTIC_EXCLUDE_VOICETASTIC
