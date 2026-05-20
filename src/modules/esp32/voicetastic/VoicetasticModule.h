#pragma once

#include "configuration.h"

#if defined(ARCH_ESP32) && !MESHTASTIC_EXCLUDE_VOICETASTIC

#include "FSCommon.h" // FSCom (LittleFS) for buffering raw PCM during capture
#include "SinglePortModule.h"
#include "VtAssembler.h"
#include "VtChunker.h"
#include "concurrency/OSThread.h"
#include "mesh/MeshTypes.h"
#include "mesh/generated/meshtastic/portnums.pb.h"
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

    // Queue an outbound message. Builds + FEC-encodes + chunks, then drives
    // the runOnce() TX loop. Returns false if another message is already in
    // flight or the audio is too large for the current modem preset.
    bool enqueueOutbound(const uint8_t *audio, size_t audio_len,
                         voicetastic::CodecId codec, uint8_t codec_param,
                         NodeNum to = NODENUM_BROADCAST, uint8_t channel = 0);

    bool isTransmitting() const { return tx_active; }

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
    bool isRecording() const { return rec_state != eRecIdle; }

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
    voicetastic::Codec2Mode getCodec2Mode() const { return codec2_mode; }
    void setCodec2Mode(voicetastic::Codec2Mode mode) { codec2_mode = mode; }

    // Mini-player API. Received voice messages no longer auto-play; instead
    // they accumulate in pending_play_queue and the chat-screen widget drives
    // playback explicitly via these calls.
    size_t   pendingPlayCount() const { return pending_play_queue.size(); }
    bool     playNextPending();              // start playing the oldest queued message
    bool     playByMessageId(uint32_t message_id); // play the queued message that matches; false if not found
    void     stopPlayback();                 // interrupt current playback (worker exits early)
    bool     isPlaying() const { return playing; }
    uint32_t playbackElapsedMs() const;      // 0 when not playing
    uint32_t playbackTotalMs() const { return play_total_ms; }
    NodeNum  playbackFromNode() const { return play_from_node; }
    uint32_t playbackMessageId() const { return play_message_id; }
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
    bool hasPending() const { return !pending_audio.empty(); }

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
    // TX state. One in-flight message at a time in this iteration.
    voicetastic::OutboundMessage tx_msg{};
    bool     tx_active = false;
    uint16_t tx_next_chunk = 0;       // 0..(total_data + parity_count - 1)
    uint32_t tx_paced_until_ms = 0;
    NodeNum  tx_to = NODENUM_BROADCAST;
    uint8_t  tx_channel = 0;          // Meshtastic channel index for the outbound message;
                                      // also drives the channel PSK lookup that keys the header MAC.
    // NACK loop state. After the linear walk completes we keep tx_msg state
    // alive for TX_NACK_LINGER_MS so we can answer inbound NACKs by replaying
    // the missing data chunks. Retransmit queue is drained ahead of the linear
    // walk in sendOneChunk; once it empties and the linger expires, tx_active
    // is cleared.
    static constexpr uint32_t TX_NACK_LINGER_MS = 8000;
    std::vector<uint8_t> tx_retransmit_idx;  // FIFO of data chunk indices to re-send
    uint32_t tx_linger_until_ms = 0;         // 0 while still inside the linear walk

    // Boot-time test broadcast: sent once a few seconds after boot to verify
    // the wire pipeline against voicetastic-desktop receivers. Phase 4 will
    // replace this with a mic-driven path.
    bool     boot_test_sent = false;
    uint32_t boot_ms = 0;

    uint8_t  stream_seq_counter = 0;

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
    uint32_t              enc_frames_total = 0; // populated when entering eRecEncoding
    uint32_t              enc_frames_done = 0;

    // "Armed" audio: Codec2 bytes captured but not yet sent. The chat screen
    // sends it explicitly via sendPending() once the user presses ENTER.
    std::vector<uint8_t>  pending_audio;

    // One-shot encoder worker task. Spawned by stopRecording with a 32 KB
    // stack; reads PCM from FSCom, runs codec2_encode frame by frame on its
    // own stack, drops the result into encoder_result_audio, and self-deletes.
    // The loop task polls encoder_done in eRecEncoding state.
    std::vector<uint8_t>  encoder_result_audio;
    volatile bool         encoder_done = false;
    TaskHandle_t          encoder_task = nullptr;

    static void encoderTaskTrampoline(void *self);
    void encoderTaskBody();

    // Playback (Phase 6). Started explicitly by playNextPending() in response
    // to a chat-screen mini-player click. Worker tears down the mic, brings
    // the DAC up, decodes codec2 frame-by-frame, writes PCM to I2S, tears the
    // DAC back down and re-inits the mic on exit.
    bool                  playing = false;
    volatile bool         playback_done = false;
    volatile bool         playback_stop_requested = false;
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
