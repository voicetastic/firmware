#pragma once

#include "configuration.h"

#if defined(ARCH_ESP32) && defined(HAS_VOICETASTIC) && !MESHTASTIC_EXCLUDE_VOICETASTIC

#include "VtProtocol.h"
#include <stdint.h>
#include <stddef.h>
#include <vector>

#if defined(HAS_SDCARD)
#include <SD.h>
#endif

// SD-backed inbox for received Voicetastic voice messages.
//
// Why: a fully-populated pending_play_queue can hold up to ~80 KB of codec2
// audio in PSRAM (16 entries × ~5 KB at mode 1200, 30s/message). That's a
// permanent resident — entries stay until evicted by the FIFO cap, even
// after they've been played, so the chat-screen mini-player can replay them.
//
// On T-Deck (HAS_SDCARD + SPI shared with TFT) we instead persist the
// reassembled audio bytes to /voicetastic_inbox/<message_id>.c2 on SD,
// keeping only the small metadata struct (ReceivedVoiceMessage) in PSRAM.
// Playback streams from SD frame-by-frame via the playback worker task.
//
// Fallback: when no SD card is mounted at boot, available() returns false
// and callers keep audio in RAM (current behaviour). The decision is fixed
// at boot — we don't switch mid-session if the user inserts a card later.

namespace voicetastic { namespace inbox {

// Did we successfully mount and prepare the SD inbox at boot? When false,
// callers should keep audio in RAM via ReceivedVoiceMessage::audio.
bool available();

// Initialise the SD inbox: probe that SD is mounted, ensure the directory
// exists, and wipe any stale messages from a previous boot. Must be called
// once during module construction (after setupSDCard()). Returns true if
// the inbox is usable for subsequent store/open calls.
bool init();

// Write a reassembled message to /voicetastic_inbox/<message_id>.c2.
// Concatenates DATA shards back-to-back; the final shard is trimmed to
// `last_data_real_size`. Returns the total bytes written, or 0 on failure
// (caller falls back to the RAM path).
size_t store(uint32_t message_id,
             const std::vector<std::vector<uint8_t>> &data_shards,
             uint8_t total_data, uint8_t chunk_size,
             size_t last_data_real_size);

// Open a previously stored message for reading. Caller must hold spiLock
// around any read on the returned File. Returns an invalid File if missing.
#if defined(HAS_SDCARD)
File open(uint32_t message_id);
#endif

// Delete a stored message (on eviction or session-end cleanup).
void erase(uint32_t message_id);

// Wipe the inbox directory. Called on boot to drop stale files from a
// previous session (we don't yet persist mini-player state across reboots).
void clearAll();

// Outbox (single-slot): a SD-backed buffer for codec2 bytes produced by the
// encoder and waiting for the user to press send. Saves up to ~55 KB of
// resident PSRAM during the typically-longest part of the outbound
// lifecycle (between "encoder done" and "user presses send"). Only one
// outbox exists at a time — there's only one outbound message in flight.

#if defined(HAS_SDCARD)
// Open the outbox for writing (truncating). Caller writes, then closes.
File outboxOpenWrite();
// Open the outbox for reading. Returns invalid File if nothing pending.
File outboxOpenRead();
#endif

// Wipe the outbox file (discardPending, post-send cleanup, boot).
void outboxClear();

}} // namespace voicetastic::inbox

#endif // ARCH_ESP32 && HAS_VOICETASTIC && !MESHTASTIC_EXCLUDE_VOICETASTIC
