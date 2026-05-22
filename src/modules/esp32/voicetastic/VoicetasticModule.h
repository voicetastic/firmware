#pragma once

#include "configuration.h"

#if defined(ARCH_ESP32) && defined(HAS_VOICETASTIC) && !MESHTASTIC_EXCLUDE_VOICETASTIC

// Voicetastic's RS-encoded outbound message can pin (total_data + parity_count)
// × chunk_size of heap simultaneously — up to ~76 KB at SHORT presets, plus
// the audio buffer, plus the encoder result. The T-Deck has 8 MB of PSRAM
// and that's where these live in practice; on a non-PSRAM board the peak
// would push us into OOM territory even with the smaller LongFast preset.
// Require an explicit PSRAM declaration from the variant so we fail fast at
// build time rather than at field-deploy.
#if !defined(BOARD_HAS_PSRAM)
#error "HAS_VOICETASTIC requires BOARD_HAS_PSRAM. Either define BOARD_HAS_PSRAM in the variant's platformio.ini, or unset HAS_VOICETASTIC."
#endif

#include "FSCommon.h" // FSCom (LittleFS) for buffering raw PCM during capture
#include "SinglePortModule.h"
#include "VtAssembler.h"
#include "VtChunker.h"
#include "concurrency/Lock.h"
#include "concurrency/LockGuard.h"
#include "concurrency/OSThread.h"
#include "mesh/MeshTypes.h"
#include "mesh/generated/meshtastic/portnums.pb.h"
#include <atomic>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <vector>

/*
 * VoicetasticModule
 *
 * Store-and-forward voice messaging over Meshtastic, wire-compatible with
 * voicetastic-desktop (https://git.cha-sam.re/acarteron/voicetastic-desktop).
 *
 * Flow on send:
 *   record (ES7210)  ->  Codec2 encode  ->  Reed-Solomon FEC
 *                    ->  16-byte v2 header + chunking
 *                    ->  PRIVATE_APP packets, paced per modem preset
 *
 * Flow on receive:
 *   PRIVATE_APP packet -> header parse + MAC verify
 *                      -> reassembly state (per from,message_id)
 *                      -> FEC recovery
 *                      -> Codec2 decode -> I2S DAC (MAX98357A) playback
 *
 * Scope of this build:
 *   - TX + RX, Codec2 only, plaintext bodies, with RS FEC.
 *   - No NACK rounds (spec v2 §3.4 / §9), no AES-GCM envelope (§7),
 *     no AMR-NB / Opus, no per-sender caps / blacklist beyond a small
 *     in-RAM ring buffer of recent messages.
 *
 * Hardware target: LilyGo T-Deck (ESP32-S3 + ES7210 + MAX98357A).
 * GPIO 21 is shared between ES7210_LRCK and DAC_I2S_MCLK so capture and
 * playback are mutually exclusive at the hardware level.
 */
class VoicetasticModule : public SinglePortModule, private concurrency::OSThread
{
  public:
    VoicetasticModule();

    // IMPORTANT: do NOT set loopbackOk = true on this module. Our state_lock_
    // is non-recursive; broadcast TX synchronously fan-outs through
    // MeshService::sendToMesh -> Router::sendLocal -> Router::handleReceived
    // -> MeshModule::callModules, and a loopback-enabled module would have its
    // handleReceived() re-entered on the same task while runOnce() still holds
    // the lock — deadlock. The default loopbackOk=false (MeshModule.h) is
    // load-bearing here.

    // Queue an outbound message. Builds + FEC-encodes + chunks, then drives
    // the runOnce() TX loop. Returns false if another message is already in
    // flight or the audio is too large for the current modem preset.
    bool enqueueOutbound(const uint8_t *audio, size_t audio_len,
                         voicetastic::CodecId codec, uint8_t codec_param,
                         NodeNum to = NODENUM_BROADCAST, uint8_t channel = 0);

    bool isTransmitting() const { concurrency::LockGuard g(&state_lock_); return tx_active; }

    // Start a mic recording of up to `duration_ms` milliseconds at Codec2 mode
    // 1200 (8 kHz, ~150 B/s). Capture + encode happen one frame per runOnce()
    // tick on the loop task (40 ms per frame), using static BSS buffers so the
    // task stack isn't loaded. When the duration elapses (or stopRecording()
    // is called), the encoded audio is *held* in pending_audio for an explicit
    // sendPending() call. Returns false if a recording or TX is already in
    // progress, or the mic / encoder fail to initialize.
    bool startRecording(uint32_t duration_ms, NodeNum to = NODENUM_BROADCAST);

    // Stop the current recording early. Audio is held; not auto-sent.
    void stopRecording();

    // True while we're actively capturing PCM OR encoding the captured PCM
    // to Codec2 frames. The chat screen treats both as "REC..." for the user.
    bool isRecording() const { concurrency::LockGuard g(&state_lock_); return rec_state != eRecIdle; }

    // Codec2 bitrate selection. Per VOICE_PROTOCOL.md §3.2.2, all modes share
    // the same wire framing — the mode is advertised in each frame's
    // codec_param. Receivers (including this firmware's playback path) decode
    // using the mode advertised on the wire, not their own outgoing setting,
    // so setting this differently on each end is safe.
    //
    // Default is M_1200 (6 B/40ms = 1.2 kbps), the most LoRa-friendly mode.
    // M_3200 produces noticeably better audio at the cost of ~5x airtime.
    // M_1600 / M_1400 / M_1300 are interpolations. Use of M_700/M_700B would
    // require updating MAX_PARITY / chunk_size guards, not exposed here.
    voicetastic::Codec2Mode getCodec2Mode() const { concurrency::LockGuard g(&state_lock_); return codec2_mode; }
    // Set and persist the encode mode in NVS so it survives reboot. The next
    // outbound recording uses the new mode; in-flight TX is unaffected.
    void setCodec2Mode(voicetastic::Codec2Mode mode);


    // Mini-player API. Received voice messages no longer auto-play; instead
    // they accumulate in pending_play_queue and the chat-screen widget drives
    // playback explicitly via these calls.
    size_t   pendingPlayCount() const { concurrency::LockGuard g(&state_lock_); return pending_play_queue.size(); }
    bool     playNextPending();              // start playing the oldest queued message
    bool     playByMessageId(uint32_t message_id); // play the queued message that matches; false if not found
    void     stopPlayback();                 // interrupt current playback (worker exits early)
    bool     isPlaying() const { concurrency::LockGuard g(&state_lock_); return playing; }
    uint32_t playbackElapsedMs() const;      // 0 when not playing
    uint32_t playbackTotalMs() const { concurrency::LockGuard g(&state_lock_); return play_total_ms; }
    NodeNum  playbackFromNode() const { concurrency::LockGuard g(&state_lock_); return play_from_node; }
    uint32_t playbackMessageId() const { concurrency::LockGuard g(&state_lock_); return play_message_id; }
    // Inspect a queued message without consuming it.
    bool     peekPending(size_t index, NodeNum &from, uint32_t &message_id,
                         uint32_t &approx_duration_ms) const;
    // Extended peek: also exposes `to` (broadcast or this node), channel
    // index, and whether the message has been played at least once. Used by
    // the chat-screen mini-player widget to route bubbles to the right chat
    // and dim already-played entries.
    bool     peekPendingFull(size_t index, NodeNum &from, NodeNum &to, uint8_t &channel,
                             uint32_t &message_id, uint32_t &approx_duration_ms,
                             bool &played) const;

    // True when there's a captured-but-unsent audio buffer waiting to be sent.
    bool hasPending() const { concurrency::LockGuard g(&state_lock_); return !pending_audio.empty(); }

    // Send the held audio to the chosen destination. Returns false if there's
    // no held audio or TX is already in progress.
    bool sendPending(NodeNum to, uint8_t channel = 0);

    // Throw away the held audio (user cancelled).
    void discardPending();

    // Milliseconds since the active recording started, or 0 if not recording.
    uint32_t recordElapsedMs() const;

  protected:
    virtual ProcessMessage handleReceived(const meshtastic_MeshPacket &mp) override;
    virtual int32_t runOnce() override;

  private:
    // Coarse-grained mutex around module state. Acquired at every public entry
    // point and at the top of runOnce()/handleReceived(). The UI/TFT task lives
    // on core 0 and calls into the IClientBase voice* hooks (via
    // VoicetasticPacketClient); runOnce() and handleReceived() are scheduled by
    // the OSThread controller on the main task. Both threads read+write the
    // play queue, the TX state machine and the recording state, so a lock is
    // mandatory — without one, an LVGL peek racing a push_back would corrupt
    // the vector.
    //
    // Worker tasks (encoder, playback) do NOT take this lock. They hand off via
    // `volatile bool` flags and own dedicated result buffers; the loop task
    // only reads those buffers after the done flag flips. Internal helpers
    // invoked from inside the lock (recordFrame, sendOneChunk, txSendShard,
    // sendNack, the *Locked variants) must NOT re-acquire — concurrency::Lock
    // is not recursive.
    mutable concurrency::Lock state_lock_;

    // Lock-assuming variants of public methods that are also called from
    // already-locked internal contexts (sendPending → enqueueOutboundLocked,
    // recordFrame → stopRecordingLocked).
    bool enqueueOutboundLocked(const uint8_t *audio, size_t audio_len,
                               voicetastic::CodecId codec, uint8_t codec_param,
                               NodeNum to, uint8_t channel);
    void stopRecordingLocked();

    // Set by the constructor if BOARD_HAS_PSRAM is declared but no PSRAM is
    // actually mapped at boot. While true, every public entry point and
    // runOnce() returns immediately so we don't try to allocate the big
    // shard / audio buffers in DRAM. Read-only after construction; no lock.
    bool psram_missing = false;

    // TX state. One in-flight message at a time in this iteration.
    voicetastic::OutboundMessage tx_msg{};
    bool     tx_active = false;
    uint16_t tx_next_chunk = 0;       // 0..(total_data + parity_count - 1)
    uint32_t tx_paced_until_ms = 0;
    NodeNum  tx_to = NODENUM_BROADCAST;
    uint8_t  tx_channel = 0;          // Meshtastic channel index for the outbound message.
                                      // The v3 header MAC is plain SHA-256 over header[0..12]
                                      // (unkeyed); per-channel confidentiality is delegated to
                                      // Meshtastic's per-packet AES-CTR.
    // NACK loop state. After the linear walk completes we keep tx_msg state
    // alive for TX_NACK_LINGER_MS so we can answer inbound NACKs by replaying
    // the missing data chunks. Retransmit queue is drained ahead of the linear
    // walk in sendOneChunk; once it empties and the linger expires, tx_active
    // is cleared.
    static constexpr uint32_t TX_NACK_LINGER_MS = 8000;
    // FIFO of data chunk indices to re-send in response to inbound NACKs.
    // We use a vector + head index rather than std::deque (heavier per-node
    // allocator overhead on libstdc++) or repeated `erase(begin())` (O(n)
    // per pop). The head advances on each retransmit; the storage is
    // squashed back to [head..end()) only when it grows large, keeping pop
    // amortized O(1). Treat `tx_retransmit_idx` as a black box — all access
    // goes through the helpers below.
    std::vector<uint8_t> tx_retransmit_idx;
    size_t               tx_retransmit_head = 0;

    bool   retransmitEmpty() const { return tx_retransmit_head >= tx_retransmit_idx.size(); }
    size_t retransmitSize() const  { return tx_retransmit_idx.size() - tx_retransmit_head; }
    void   retransmitClear() { tx_retransmit_idx.clear(); tx_retransmit_head = 0; }
    uint8_t retransmitPop() {
        const uint8_t v = tx_retransmit_idx[tx_retransmit_head++];
        // Compact when the dead prefix dominates the live tail; keeps the
        // backing storage from growing unboundedly under long NACK loops.
        if (tx_retransmit_head > 16 && tx_retransmit_head * 2 >= tx_retransmit_idx.size()) {
            tx_retransmit_idx.erase(tx_retransmit_idx.begin(),
                                    tx_retransmit_idx.begin() + tx_retransmit_head);
            tx_retransmit_head = 0;
        }
        return v;
    }
    bool retransmitContains(uint8_t i) const {
        for (size_t k = tx_retransmit_head; k < tx_retransmit_idx.size(); ++k) {
            if (tx_retransmit_idx[k] == i) return true;
        }
        return false;
    }
    void retransmitPush(uint8_t i) { tx_retransmit_idx.push_back(i); }
    uint32_t tx_linger_until_ms = 0;         // 0 while still inside the linear walk

    // Boot-time test broadcast: sent once a few seconds after boot to verify
    // the wire pipeline against voicetastic-desktop receivers. Phase 4 will
    // replace this with a mic-driven path.
    bool     boot_test_sent = false;
    uint32_t boot_ms = 0;

    uint8_t  stream_seq_counter = 0;
    // Reservation boundary: stream_seq_counter may consume values up to (but
    // not including) stream_seq_reserved_end without touching NVS. When it
    // hits the boundary, we write the next block's end to NVS and advance.
    // Initialised by the constructor's NVS-resume path; never wraps explicitly
    // — uint8_t arithmetic handles the 256-wrap fine since we only compare
    // counter == reserved_end as the trigger.
    uint8_t  stream_seq_reserved_end = 0;

    // Active outbound codec2 mode. Initialised from the build flag
    // VOICETASTIC_CODEC2_MODE (default M_1200) and mutable at runtime via
    // setCodec2Mode(); receivers always honour each frame's codec_param so
    // the two ends don't need to agree out-of-band.
#ifndef VOICETASTIC_CODEC2_MODE
#define VOICETASTIC_CODEC2_MODE voicetastic::Codec2Mode::M_1200
#endif
    voicetastic::Codec2Mode codec2_mode = VOICETASTIC_CODEC2_MODE;

    // SD-buffered recording state.
    //
    //   eRecIdle      — nothing in flight.
    //   eRecRecording — writing raw PCM to /voice.pcm on SD, one frame per tick.
    //   eRecEncoding  — reading PCM back from SD and encoding to Codec2 in
    //                   pending_audio, one frame per tick (spaced so the loop
    //                   task has breathing room between codec2_encode calls).
    enum RecState : uint8_t { eRecIdle = 0, eRecRecording, eRecEncoding };
    RecState              rec_state = eRecIdle;
    uint32_t              rec_started_ms = 0;
    uint32_t              rec_duration_ms = 0;
    File                  rec_pcm_file;        // open during eRecRecording (write) and eRecEncoding (read)
    uint32_t              rec_frame_count = 0;  // PCM frames captured this recording (reset in startRecording)
    uint32_t              enc_frames_total = 0; // populated when entering eRecEncoding
    uint32_t              enc_frames_done = 0;

    // "Armed" audio: Codec2 bytes captured but not yet sent. The chat screen
    // sends it explicitly via sendPending() once the user presses ENTER.
    std::vector<uint8_t>  pending_audio;

    // One-shot encoder worker task. Spawned by stopRecording with a 32 KB
    // stack; reads PCM from FSCom, runs codec2_encode frame by frame on its
    // own stack, drops the result into encoder_result_audio, and self-deletes.
    // The loop task polls encoder_done in eRecEncoding state.
    //
    // encoder_done is std::atomic so the encoder task's store and the loop
    // task's load are properly ordered against the surrounding writes /
    // reads of encoder_result_audio. `volatile` (the previous type) is not
    // a memory barrier under the C++ memory model — the compiler is free to
    // sink encoder_result_audio writes past the volatile store, with no
    // diagnostic. std::atomic with release/acquire prevents that.
    std::vector<uint8_t>  encoder_result_audio;
    std::atomic<bool>     encoder_done{false};
    TaskHandle_t          encoder_task = nullptr;

    static void encoderTaskTrampoline(void *self);
    void encoderTaskBody();

    // Playback (Phase 6). Started explicitly by playNextPending() in response
    // to a chat-screen mini-player click. Worker tears down the mic, brings
    // the DAC up, decodes codec2 frame-by-frame, writes PCM to I2S, tears the
    // DAC back down and re-inits the mic on exit.
    // Cross-call PCM accumulator for the on-disk recording. `i2s_read` is
    // free to return short of the requested sample count when the requested
    // wall-time exceeds the timeout, so we accumulate samples across runOnce
    // ticks until we have a full frame, then flush to LittleFS. Without
    // this, a 30 ms i2s_read timeout against a 40 ms-wall-time request
    // would silently drop every frame.
    int16_t  rec_pcm_accum[320];   // VT_PCM_SAMPLES_PER_FRAME; static-sized to keep off the stack
    uint16_t rec_pcm_accum_n = 0;  // current fill level in samples (0..320)

    bool                  playing = false;
    // Same rationale as encoder_done: cross-task signaling needs std::atomic
    // for the C++ memory model to guarantee ordering against the worker's
    // writes to playback_msg / play_started_ms.
    std::atomic<bool>     playback_done{false};
    std::atomic<bool>     playback_stop_requested{false};
    TaskHandle_t          playback_task = nullptr;
    voicetastic::ReceivedVoiceMessage playback_msg;
    uint32_t              play_started_ms = 0;   // millis() when worker started writing PCM
    uint32_t              play_total_ms = 0;     // duration of currently-playing or queued-front message
    NodeNum               play_from_node = 0;
    uint32_t              play_message_id = 0;   // identifies which queued bubble the UI should mark as playing

    // Mini-player inbox: received voice messages, capped at kMaxInbox. Entries
    // persist after playback (each carries a `played` flag) so the chat-screen
    // widget can replay them. When the cap is hit, the oldest entry is dropped
    // FIFO to bound RAM usage.
    static constexpr size_t kMaxInbox = 16;
    std::vector<voicetastic::ReceivedVoiceMessage> pending_play_queue;

    static void playbackTaskTrampoline(void *self);
    void playbackTaskBody();
    bool startPlayback(const voicetastic::ReceivedVoiceMessage &msg);

    // Inbound assembler: parses v2 frames into reassembled voice messages.
    voicetastic::VtAssembler assembler;

    void sendOneChunk();
    // Re-send a specific DATA shard (used for NACK-driven retransmits). Same
    // header/MAC/body path as sendOneChunk; chunk_index is the data index
    // (0..total_data-1).
    void sendDataRetransmit(uint8_t data_idx);
    // Build + send one DATA or PARITY frame from tx_msg state. Shared between
    // the linear-walk path (sendOneChunk) and the NACK retransmit path
    // (sendDataRetransmit).
    void txSendShard(bool is_data, uint8_t idx, bool last_in_stream);
    // Send a NACK frame from the receiver side. Echoes the originating
    // message's identity fields per spec §3.4.
    void sendNack(const voicetastic::VtAssembler::PendingNack &nk);
    void recordFrame();   // PCM read -> FSCom write; called from runOnce in eRecRecording
};

extern VoicetasticModule *voicetasticModule;

#endif // ARCH_ESP32 && !MESHTASTIC_EXCLUDE_VOICETASTIC
