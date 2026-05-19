#pragma once

#include "configuration.h"

#if defined(ARCH_ESP32) && !MESHTASTIC_EXCLUDE_VOICETASTIC

#include "SinglePortModule.h"
#include "VtChunker.h"
#include "concurrency/Lock.h"
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

    // Queue an outbound message. Builds + FEC-encodes + chunks, then drives
    // the runOnce() TX loop. Returns false if another message is already in
    // flight or the audio is too large for the current modem preset.
    bool enqueueOutbound(const uint8_t *audio, size_t audio_len,
                         voicetastic::CodecId codec, uint8_t codec_param,
                         NodeNum to = NODENUM_BROADCAST);

    bool isTransmitting() const { return tx_active; }

    // Start a mic recording of up to `duration_ms` milliseconds at Codec2 mode
    // 1200 (8 kHz, ~150 B/s). The actual capture + encode runs on a dedicated
    // FreeRTOS task so the cooperative loopTask (which shares space with
    // LVGL + LovyanGFX) doesn't overflow its stack. When the duration elapses
    // (or stopRecording() is called), the encoded audio is handed back to the
    // loop task which calls enqueueOutbound() on its next tick.
    // Returns false if a recording or TX is already in progress.
    bool startRecording(uint32_t duration_ms, NodeNum to = NODENUM_BROADCAST);

    // Stop the current recording early; the captured-so-far audio is still sent.
    void stopRecording();

    bool isRecording() const { return recording.load(std::memory_order_acquire); }

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

    // Recording state. The task body sets `recording` true on entry and false
    // on exit. Loop task reads only; codec2 task writes only.
    std::atomic<bool>     recording{false};        // task is actively capturing
    std::atomic<bool>     rec_stop_requested{false}; // loop task asks task to bail early
    std::atomic<bool>     rec_audio_ready{false};  // task has placed audio in rec_audio_done
    uint32_t              rec_duration_ms = 0;     // set by loop task before kicking the task
    NodeNum               rec_to_pending = NODENUM_BROADCAST;
    std::vector<uint8_t>  rec_audio_done;          // filled by codec2 task; consumed by loop task
    concurrency::Lock     rec_audio_lock;          // protects rec_audio_done across the hand-off

    TaskHandle_t          codec2_task = nullptr;

    static void codec2TaskTrampoline(void *self);
    void codec2TaskBody();

    void sendOneChunk();
    void finishRecordingHandoff(uint32_t now);
};

extern VoicetasticModule *voicetasticModule;

#endif // ARCH_ESP32 && !MESHTASTIC_EXCLUDE_VOICETASTIC
