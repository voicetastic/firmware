#include "VtInbox.h"

#if defined(ARCH_ESP32) && defined(HAS_VOICETASTIC) && !MESHTASTIC_EXCLUDE_VOICETASTIC

#include "SPILock.h"
#include "concurrency/LockGuard.h"
#include "configuration.h"
#include <stdio.h>
#include <string.h>

namespace voicetastic { namespace inbox {

static constexpr const char *kInboxDir = "/voicetastic_inbox";
static constexpr const char *kOutboxPath = "/voicetastic_outbox.c2";

// State set by init(). True once we've confirmed SD is mounted and the
// inbox directory exists. False means callers should fall back to RAM.
static bool s_available = false;

#if !defined(HAS_SDCARD)
// Builds without SD-card hardware just stub everything to "unavailable" so
// the calling sites compile but always take the RAM path.
bool available() { return false; }
bool init() { return false; }
size_t store(uint32_t, const std::vector<std::vector<uint8_t>> &, uint8_t, uint8_t, size_t) { return 0; }
void erase(uint32_t) {}
void clearAll() {}
void outboxClear() {}
#else

// Build the per-message path. Buffer big enough for the prefix + 8 hex digits
// + ".c2" + NUL.
static void makePath(uint32_t message_id, char *out, size_t cap)
{
    snprintf(out, cap, "%s/%08x.c2", kInboxDir, (unsigned)message_id);
}

bool available() { return s_available; }

bool init()
{
    // SD.cardType() returns CARD_NONE when no card is mounted; we read it
    // under spiLock since the TFT task may be mid-transaction on the shared
    // SPI bus.
    concurrency::LockGuard g(spiLock);
    if (SD.cardType() == CARD_NONE) {
        LOG_INFO("Voicetastic inbox: SD not mounted; using RAM for received messages");
        s_available = false;
        return false;
    }
    if (!SD.exists(kInboxDir)) {
        if (!SD.mkdir(kInboxDir)) {
            LOG_WARN("Voicetastic inbox: mkdir %s failed; falling back to RAM", kInboxDir);
            s_available = false;
            return false;
        }
    }
    s_available = true;
    LOG_INFO("Voicetastic inbox: SD-backed at %s", kInboxDir);
    return true;
}

size_t store(uint32_t message_id,
             const std::vector<std::vector<uint8_t>> &data_shards,
             uint8_t total_data, uint8_t chunk_size,
             size_t last_data_real_size)
{
    if (!s_available) return 0;
    if (total_data == 0) return 0;

    char path[40];
    makePath(message_id, path, sizeof(path));

    concurrency::LockGuard g(spiLock);
    // Overwrite any prior file at this message_id; v3 message_ids are random
    // u32 so collisions are vanishingly rare, but a duplicate caused by a
    // sender retransmit after a previous evict-and-recreate cycle should
    // not get a stale tail appended.
    File f = SD.open(path, FILE_WRITE);
    if (!f) {
        LOG_WARN("Voicetastic inbox: open(%s) for write failed", path);
        return 0;
    }
    size_t written = 0;
    for (uint8_t i = 0; i < total_data; ++i) {
        const std::vector<uint8_t> &shard = data_shards[i];
        const size_t take = (i == total_data - 1) ? last_data_real_size : (size_t)chunk_size;
        const size_t got = f.write(shard.data(), take);
        if (got != take) {
            LOG_ERROR("Voicetastic inbox: short write at shard %u (%u/%u)", (unsigned)i,
                      (unsigned)got, (unsigned)take);
            f.close();
            SD.remove(path);
            return 0;
        }
        written += got;
    }
    f.close();
    return written;
}

File open(uint32_t message_id)
{
    if (!s_available) return File();
    char path[40];
    makePath(message_id, path, sizeof(path));
    concurrency::LockGuard g(spiLock);
    return SD.open(path, FILE_READ);
}

void erase(uint32_t message_id)
{
    if (!s_available) return;
    char path[40];
    makePath(message_id, path, sizeof(path));
    concurrency::LockGuard g(spiLock);
    if (SD.exists(path)) SD.remove(path);
}

void clearAll()
{
    if (!s_available) return;
    concurrency::LockGuard g(spiLock);
    File dir = SD.open(kInboxDir);
    if (!dir || !dir.isDirectory()) {
        if (dir) dir.close();
        return;
    }
    File entry;
    while ((entry = dir.openNextFile())) {
        if (!entry.isDirectory()) {
            // SD's name() returns just the filename (no path) on some core
            // versions and a leading slash + path on others; rebuild the
            // canonical path to remove.
            const char *name = entry.name();
            const char *base = strrchr(name, '/');
            base = base ? base + 1 : name;
            char path[40];
            snprintf(path, sizeof(path), "%s/%s", kInboxDir, base);
            entry.close();
            SD.remove(path);
        } else {
            entry.close();
        }
    }
    dir.close();
    // Also nuke any leftover outbox from a prior session.
    if (SD.exists(kOutboxPath)) SD.remove(kOutboxPath);
}

File outboxOpenWrite()
{
    if (!s_available) return File();
    concurrency::LockGuard g(spiLock);
    if (SD.exists(kOutboxPath)) SD.remove(kOutboxPath);
    return SD.open(kOutboxPath, FILE_WRITE);
}

File outboxOpenRead()
{
    if (!s_available) return File();
    concurrency::LockGuard g(spiLock);
    return SD.open(kOutboxPath, FILE_READ);
}

void outboxClear()
{
    if (!s_available) return;
    concurrency::LockGuard g(spiLock);
    if (SD.exists(kOutboxPath)) SD.remove(kOutboxPath);
}
#endif // HAS_SDCARD

}} // namespace voicetastic::inbox

#endif // ARCH_ESP32 && HAS_VOICETASTIC && !MESHTASTIC_EXCLUDE_VOICETASTIC
