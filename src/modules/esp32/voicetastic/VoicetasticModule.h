#pragma once

#include "configuration.h"

#if defined(ARCH_ESP32) && !MESHTASTIC_EXCLUDE_VOICETASTIC

#include "FSCommon.h" // FSCom (LittleFS) for buffering raw PCM during capture
#include "SinglePortModule.h"
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
                         NodeNum to = NODENUM_BROADCAST);

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

    // Boot-time test broadcast: sent once a few seconds after boot to verify
    // the wire pipeline against voicetastic-desktop receivers. Phase 4 will
    // replace this with a mic-driven path.
    bool     boot_test_sent = false;
    uint32_t boot_ms = 0;

    uint8_t  stream_seq_counter = 0;

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

    void sendOneChunk();
    void recordFrame();   // PCM read -> FSCom write; called from runOnce in eRecRecording
};

extern VoicetasticModule *voicetasticModule;

#endif // ARCH_ESP32 && !MESHTASTIC_EXCLUDE_VOICETASTIC
