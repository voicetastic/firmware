#include "VoicetasticModule.h"

#if defined(ARCH_ESP32) && !MESHTASTIC_EXCLUDE_VOICETASTIC

#include "MeshService.h"
#include "NodeDB.h"
#include "RadioInterface.h"
#include "Router.h"
#include "mesh/Channels.h"

namespace {

// Resolve the AES-key bytes that Meshtastic would use to encrypt this channel.
// Mirrors Channels::getKey() (which is private in upstream): handles short-form
// PSK expansion (1-byte index → defaultpsk + bump), secondary-without-key
// fallback to primary, and zero-padding of short user-provided keys. Returns
// true and fills `out` with up to 32 bytes; `out_len = 0` means the channel is
// in plaintext mode ("User disabled encryption"). On disabled / invalid
// channels returns false.
//
// Used as the HMAC-SHA256 key for the v2 header MAC (spec §3 / §7) so we stay
// bit-compatible with voicetastic-desktop, which keys the same way.
bool resolveChannelPsk(uint8_t channel, uint8_t out[32], size_t &out_len)
{
    const meshtastic_Channel &ch = channels.getByIndex(channel);
    if (!ch.has_settings || ch.role == meshtastic_Channel_Role_DISABLED) {
        out_len = 0;
        return false;
    }
    const auto &ps = ch.settings.psk;
    memset(out, 0, 32);

    if (ps.size == 0) {
        if (ch.role == meshtastic_Channel_Role_SECONDARY) {
            // Inherit primary channel's key.
            return resolveChannelPsk((uint8_t)channels.getPrimaryIndex(), out, out_len);
        }
        out_len = 0; // encryption disabled
        return true;
    }
    if (ps.size == 1) {
        const uint8_t pskIndex = ps.bytes[0];
        if (pskIndex == 0) {
            out_len = 0; // encryption disabled via short index 0
            return true;
        }
        memcpy(out, defaultpsk, sizeof(defaultpsk));
        // Per Channels::getKey: short index 1 = unchanged defaultpsk; higher
        // indices bump the last byte.
        out[sizeof(defaultpsk) - 1] = (uint8_t)(out[sizeof(defaultpsk) - 1] + pskIndex - 1);
        out_len = sizeof(defaultpsk);
        return true;
    }
    const size_t n = ps.size <= 32 ? (size_t)ps.size : 32;
    memcpy(out, ps.bytes, n);
    if (n < 16)               out_len = 16;       // AES128 zero-pad
    else if (n < 32 && n != 16) out_len = 32;    // AES256 zero-pad
    else                        out_len = n;
    return true;
}

} // namespace
#include "VtAudio.h"
#include "VtProtocol.h"
#include "concurrency/LockGuard.h"
#include "rs/rs.h"
#include <Arduino.h>
#include <esp_random.h>
#include <string.h>

VoicetasticModule *voicetasticModule;

// Round-trip a known header through the codec at boot so any wire-format
// regression shows up immediately in the serial log. Cheap; runs once.
static void runProtocolSelfTest()
{
    using namespace voicetastic;
    VtHeader h{};
    h.version        = PROTOCOL_VERSION;
    h.packet_type    = PacketType::DATA;
    h.encrypted      = false;
    h.last_in_stream = false;
    h.mac_keyed      = false;
    h.message_id     = 0xDEADBEEF;
    h.codec          = CodecId::CODEC2;
    h.codec_param    = (uint8_t)Codec2Mode::M_1200; // self-test only; any valid mode is fine
    h.stream_seq     = 7;
    h.chunk_index    = 3;
    h.total_data     = 22;
    h.parity_count   = 5;

    uint8_t buf[HEADER_SIZE] = {0};
    if (encodeHeader(h, buf) != HEADER_SIZE) {
        LOG_ERROR("Voicetastic selftest: encodeHeader failed");
        return;
    }

    VtHeader r{};
    if (!decodeHeader(buf, r)) {
        LOG_ERROR("Voicetastic selftest: decodeHeader rejected our own output");
        return;
    }

    const bool ok =
        r.version == h.version && r.packet_type == h.packet_type && r.encrypted == h.encrypted &&
        r.last_in_stream == h.last_in_stream && r.message_id == h.message_id && r.codec == h.codec &&
        r.codec_param == h.codec_param && r.stream_seq == h.stream_seq && r.chunk_index == h.chunk_index &&
        r.total_data == h.total_data && r.parity_count == h.parity_count;

    if (ok) {
        LOG_INFO("Voicetastic selftest: header codec OK (mid=%08x)", (unsigned)r.message_id);
    } else {
        LOG_ERROR("Voicetastic selftest: round-trip mismatch");
    }
}

VoicetasticModule::VoicetasticModule()
    : SinglePortModule("Voicetastic", meshtastic_PortNum_PRIVATE_APP), concurrency::OSThread("Voicetastic")
{
    LOG_INFO("Voicetastic module init (port=%d, scope=TX+RX Codec2 plaintext+FEC)", (int)meshtastic_PortNum_PRIVATE_APP);
    runProtocolSelfTest();
    voicetastic::rs::runSelfTest();
    boot_ms = millis();
}

bool VoicetasticModule::enqueueOutbound(const uint8_t *audio, size_t audio_len,
                                        voicetastic::CodecId codec, uint8_t codec_param,
                                        NodeNum to, uint8_t channel)
{
    using namespace voicetastic;
    if (tx_active) {
        LOG_WARN("Voicetastic: TX busy, drop enqueue");
        return false;
    }
    const auto preset = (meshtastic_Config_LoRaConfig_ModemPreset)config.lora.modem_preset;
    const uint32_t mid = esp_random() | 1u; // non-zero u32 per spec §6
    const uint8_t  seq = stream_seq_counter++;

    if (!buildOutbound(audio, audio_len, preset, codec, codec_param, mid, seq, tx_msg)) {
        LOG_ERROR("Voicetastic: buildOutbound failed (audio_len=%u preset=%d)",
                  (unsigned)audio_len, (int)preset);
        return false;
    }

    tx_to = to;
    tx_channel = channel;
    tx_retransmit_idx.clear();
    tx_linger_until_ms = 0;
    tx_next_chunk = 0;
    tx_paced_until_ms = 0;
    tx_active = true;
    LOG_INFO("Voicetastic: enqueue mid=%08x total_data=%u parity=%u chunk=%u last_real=%u",
             (unsigned)tx_msg.message_id, (unsigned)tx_msg.total_data,
             (unsigned)tx_msg.parity_count, (unsigned)tx_msg.chunk_size,
             (unsigned)tx_msg.last_data_real_size);
    return true;
}

// Fixed PCM frame size used for on-disk buffering: 320 samples (40 ms at
// 8 kHz) per frame, 2 bytes/sample = 640 bytes per frame. Matches Codec2's
// samples_per_frame so the encoding loop reads exactly one frame at a time.
static constexpr int VT_PCM_SAMPLES_PER_FRAME = 320;
static constexpr size_t VT_PCM_BYTES_PER_FRAME = VT_PCM_SAMPLES_PER_FRAME * sizeof(int16_t);
static constexpr const char *VT_PCM_PATH = "/voicetastic.pcm";

// We buffer PCM in LittleFS (the firmware's internal filesystem, mounted at
// boot via FSCom). Guard returns true as long as FSCom has at least *some*
// addressable space — i.e. the FS is mounted. If LittleFS failed to mount
// (rare, would log "FS mount failed" earlier in boot), recording is a no-op.
static bool vtStorageAvailable()
{
    return FSCom.totalBytes() > 0;
}

bool VoicetasticModule::startRecording(uint32_t duration_ms, NodeNum to)
{
    using namespace voicetastic;
    (void)to; // Destination is decided at sendPending() time.
    if (rec_state != eRecIdle) { LOG_WARN("Voicetastic: already recording/encoding"); return false; }
    if (tx_active) { LOG_WARN("Voicetastic: TX in progress; cannot record yet"); return false; }

    // Guard: voice recording needs an SD card mounted (we buffer raw PCM there
    // to keep codec2_encode out of the loop task during capture). Without one,
    // SYM+0 is silently a no-op.
    if (!vtStorageAvailable()) {
        LOG_WARN("Voicetastic: FSCom storage not mounted; recording disabled");
        return false;
    }

    if (!pending_audio.empty()) discardPending();

    if (!VtAudio::initMic()) {
        LOG_ERROR("Voicetastic: mic init failed");
        return false;
    }

    // Truncate any previous capture and open the PCM file for writing.
    if (FSCom.exists(VT_PCM_PATH)) FSCom.remove(VT_PCM_PATH);
    rec_pcm_file = FSCom.open(VT_PCM_PATH, FILE_WRITE);
    if (!rec_pcm_file) {
        LOG_ERROR("Voicetastic: FSCom open(%s) for write failed", VT_PCM_PATH);
        VtAudio::deinitMic();
        return false;
    }

    rec_duration_ms = duration_ms;
    rec_started_ms  = millis();
    rec_state       = eRecRecording;
    LOG_INFO("Voicetastic: recording started (%u ms) to %s", (unsigned)duration_ms, VT_PCM_PATH);
    return true;
}

uint32_t VoicetasticModule::recordElapsedMs() const
{
    if (rec_state != eRecRecording) return 0;
    return millis() - rec_started_ms;
}

bool VoicetasticModule::sendPending(NodeNum to, uint8_t channel)
{
    using namespace voicetastic;
    if (pending_audio.empty()) return false;
    if (tx_active) { LOG_WARN("Voicetastic: TX busy; cannot send pending audio yet"); return false; }

    std::vector<uint8_t> audio = std::move(pending_audio);
    pending_audio.clear();
    pending_audio.shrink_to_fit();
    if (audio.empty()) return false;
    return enqueueOutbound(audio.data(), audio.size(),
                           CodecId::CODEC2, (uint8_t)codec2_mode, to, channel);
}

void VoicetasticModule::discardPending()
{
    pending_audio.clear();
    pending_audio.shrink_to_fit();
}

void VoicetasticModule::stopRecording()
{
    using namespace voicetastic;
    if (rec_state != eRecRecording) return;

    // Close the write handle and transition into the encoding phase.
    if (rec_pcm_file) {
        rec_pcm_file.flush();
        rec_pcm_file.close();
    }
    const size_t pcm_bytes = FSCom.exists(VT_PCM_PATH) ? (size_t)FSCom.open(VT_PCM_PATH, FILE_READ).size() : 0;
    enc_frames_total = (uint32_t)(pcm_bytes / VT_PCM_BYTES_PER_FRAME);
    enc_frames_done  = 0;
    LOG_INFO("Voicetastic: captured %u PCM bytes (%u frames); encoding to Codec2 mode ord=%u",
             (unsigned)pcm_bytes, (unsigned)enc_frames_total, (unsigned)codec2_mode);

    // NB: do NOT call VtAudio::deinitMic() here. Tearing the ES7210 down via
    // I2C while the LVGL keyboard task is polling the TCA8418 on the same
    // bus reliably faults the device on hardware. The mic stays resident for
    // the lifetime of the firmware; cost is ~22 KB of RAM, which we have.

    if (enc_frames_total == 0) {
        // Nothing captured (e.g. user stopped immediately). Bail back to idle.
        if (FSCom.exists(VT_PCM_PATH)) FSCom.remove(VT_PCM_PATH);
        rec_state = eRecIdle;
        return;
    }
    if (!VtAudio::initEncoder(codec2_mode)) {
        LOG_ERROR("Voicetastic: codec2 init failed at encoding phase");
        if (FSCom.exists(VT_PCM_PATH)) FSCom.remove(VT_PCM_PATH);
        rec_state = eRecIdle;
        return;
    }
    rec_pcm_file = FSCom.open(VT_PCM_PATH, FILE_READ);
    if (!rec_pcm_file) {
        LOG_ERROR("Voicetastic: FSCom reopen(%s) for read failed", VT_PCM_PATH);
        VtAudio::deinitEncoder();
        rec_state = eRecIdle;
        return;
    }
    pending_audio.clear();

    // Spawn the dedicated encoder worker task. 32 KB stack is generous — gives
    // codec2_encode's FFT and LSP scratch all the headroom it needs without
    // competing with the loop task's stack. Priority 1 = same as the TFT task
    // so we don't preempt LVGL. The task self-deletes when it finishes the
    // file and signals back via encoder_done.
    encoder_done = false;
    encoder_result_audio.clear();
    encoder_result_audio.reserve((size_t)enc_frames_total * 8);
    const BaseType_t rc = xTaskCreate(&VoicetasticModule::encoderTaskTrampoline,
                                      "vtEncode", 32768, this, 1, &encoder_task);
    if (rc != pdPASS) {
        LOG_ERROR("Voicetastic: failed to create encoder task (rc=%d)", (int)rc);
        encoder_task = nullptr;
        rec_pcm_file.close();
        FSCom.remove(VT_PCM_PATH);
        VtAudio::deinitEncoder();
        rec_state = eRecIdle;
        return;
    }
    LOG_INFO("Voicetastic: encoder worker task started");
    rec_state = eRecEncoding;
}

void VoicetasticModule::encoderTaskTrampoline(void *self)
{
    static_cast<VoicetasticModule *>(self)->encoderTaskBody();
}

// Look up Codec2 mode's bytes/frame + samples/frame without touching the
// encoder/decoder instances (used to estimate playback duration before
// allocating the worker task).
static void vtCodec2FrameInfo(voicetastic::Codec2Mode mode, int &bytes_per_frame,
                              int &samples_per_frame)
{
    using namespace voicetastic;
    switch (mode) {
    case Codec2Mode::M_3200: bytes_per_frame = 8; samples_per_frame = 160; break;
    case Codec2Mode::M_2400: bytes_per_frame = 6; samples_per_frame = 160; break;
    case Codec2Mode::M_1600: bytes_per_frame = 8; samples_per_frame = 320; break;
    case Codec2Mode::M_1400: bytes_per_frame = 7; samples_per_frame = 320; break;
    case Codec2Mode::M_1300: bytes_per_frame = 7; samples_per_frame = 320; break;
    case Codec2Mode::M_1200: bytes_per_frame = 6; samples_per_frame = 320; break;
    default: bytes_per_frame = 6; samples_per_frame = 320; break;
    }
}

static uint32_t vtEstimateDurationMs(const voicetastic::ReceivedVoiceMessage &msg)
{
    int bpf = 6, spf = 320;
    if (msg.codec == voicetastic::CodecId::CODEC2)
        vtCodec2FrameInfo((voicetastic::Codec2Mode)msg.codec_param, bpf, spf);
    if (bpf == 0) return 0;
    const uint32_t frames = (uint32_t)(msg.audio.size() / (size_t)bpf);
    return (frames * (uint32_t)spf) / 8; // samples_per_frame * 1000 / 8000 simplified
}

bool VoicetasticModule::startPlayback(const voicetastic::ReceivedVoiceMessage &msg)
{
    using namespace voicetastic;
    if (playing) return false;
    if (rec_state != eRecIdle) return false;     // can't play while recording/encoding
    if (msg.audio.empty()) return false;

    playback_msg = msg; // copy in (vector copy)
    playback_done = false;
    playback_stop_requested = false;
    play_from_node = msg.from;
    play_message_id = msg.message_id;
    play_total_ms = vtEstimateDurationMs(msg);
    play_started_ms = 0; // worker stamps this when it begins writing PCM

    const BaseType_t rc = xTaskCreate(&VoicetasticModule::playbackTaskTrampoline,
                                      "vtPlay", 32768, this, 1, &playback_task);
    if (rc != pdPASS) {
        LOG_ERROR("Voicetastic: failed to create playback task (rc=%d)", (int)rc);
        playback_task = nullptr;
        playback_msg.audio.clear();
        play_total_ms = 0;
        play_from_node = 0;
        play_message_id = 0;
        return false;
    }
    playing = true;
    LOG_INFO("Voicetastic: playback started mid=%08x (%u bytes, codec=%u/%u, ~%u ms)",
             (unsigned)msg.message_id, (unsigned)msg.audio.size(),
             (unsigned)msg.codec, (unsigned)msg.codec_param,
             (unsigned)play_total_ms);
    return true;
}

bool VoicetasticModule::playNextPending()
{
    if (pending_play_queue.empty()) return false;
    if (playing) return false;
    if (rec_state != eRecIdle) return false;
    if (tx_active) return false;
    // Play the first unplayed entry. Copy (don't move) so the audio stays in
    // the inbox for re-play by the chat-screen mini-player; mark as played.
    for (auto &m : pending_play_queue) {
        if (m.played) continue;
        voicetastic::ReceivedVoiceMessage copy = m;
        m.played = true;
        return startPlayback(copy);
    }
    // All already played: fall back to playing the oldest.
    voicetastic::ReceivedVoiceMessage copy = pending_play_queue.front();
    pending_play_queue.front().played = true;
    return startPlayback(copy);
}

bool VoicetasticModule::playByMessageId(uint32_t message_id)
{
    if (playing) return false;
    if (rec_state != eRecIdle) return false;
    if (tx_active) return false;
    for (auto &m : pending_play_queue) {
        if (m.message_id == message_id) {
            voicetastic::ReceivedVoiceMessage copy = m;
            m.played = true;
            return startPlayback(copy);
        }
    }
    return false;
}

void VoicetasticModule::stopPlayback()
{
    if (!playing) return;
    playback_stop_requested = true;
}

uint32_t VoicetasticModule::playbackElapsedMs() const
{
    if (!playing || play_started_ms == 0) return 0;
    return millis() - play_started_ms;
}

bool VoicetasticModule::peekPending(size_t index, NodeNum &from, uint32_t &message_id,
                                    uint32_t &approx_duration_ms) const
{
    if (index >= pending_play_queue.size()) return false;
    const auto &m = pending_play_queue[index];
    from = m.from;
    message_id = m.message_id;
    approx_duration_ms = vtEstimateDurationMs(m);
    return true;
}

bool VoicetasticModule::peekPendingFull(size_t index, NodeNum &from, NodeNum &to,
                                        uint8_t &channel, uint32_t &message_id,
                                        uint32_t &approx_duration_ms, bool &played) const
{
    if (index >= pending_play_queue.size()) return false;
    const auto &m = pending_play_queue[index];
    from = m.from;
    to = m.to;
    channel = m.channel;
    message_id = m.message_id;
    approx_duration_ms = vtEstimateDurationMs(m);
    played = m.played;
    return true;
}

void VoicetasticModule::playbackTaskTrampoline(void *self)
{
    static_cast<VoicetasticModule *>(self)->playbackTaskBody();
}

void VoicetasticModule::playbackTaskBody()
{
    using namespace voicetastic;

    // We need the DAC's I2S MCLK on GPIO 21, which is shared with ES7210_LRCK.
    // Release the mic before bringing the DAC up. If the mic deinit reboots
    // the device, we'll find out here and need a different strategy.
    VtAudio::deinitMic();

    if (!VtAudio::initDac()) {
        LOG_ERROR("vtPlay: DAC init failed");
        playback_done = true;
        playback_task = nullptr;
        vTaskDelete(NULL);
        return;
    }
    if (!VtAudio::initDecoder((Codec2Mode)playback_msg.codec_param)) {
        LOG_ERROR("vtPlay: codec2 decoder init failed");
        VtAudio::deinitDac();
        playback_done = true;
        playback_task = nullptr;
        vTaskDelete(NULL);
        return;
    }

    const int samples_per_frame = VtAudio::samplesPerDecodedFrame();
    const int bytes_per_frame   = VtAudio::bytesPerDecodedFrame();

    int16_t pcm[640];   // ample for any Codec2 mode (max 320 @ mode 1200)
    const uint8_t *src = playback_msg.audio.data();
    const size_t   total = playback_msg.audio.size();
    size_t         consumed = 0;
    uint32_t       frame_count = 0;

    play_started_ms = millis();
    while (consumed + bytes_per_frame <= total) {
        if (playback_stop_requested) {
            LOG_INFO("vtPlay: stop requested, exiting at frame %u", (unsigned)frame_count);
            break;
        }
        VtAudio::decodeFrame(src + consumed, pcm);
        VtAudio::writePcm(pcm, samples_per_frame);
        consumed += bytes_per_frame;
        frame_count++;
        if ((frame_count % 25) == 0) {
            LOG_DEBUG("vtPlay: decoded %u frames (%u/%u bytes)",
                      (unsigned)frame_count, (unsigned)consumed, (unsigned)total);
        }
    }

    // Drain the DMA before tearing the DAC down.
    vTaskDelay(pdMS_TO_TICKS(100));
    VtAudio::deinitDecoder();
    VtAudio::deinitDac();
    // VtAudio::deinitDac() now resets the shared GPIO 21 (and the other DAC
    // pins) back to INPUT, so the mic's I2S_NUM_1 install can claim LRCK
    // cleanly. A short settle pause makes the transition reliable on the
    // legacy i2s_legacy driver.
    vTaskDelay(pdMS_TO_TICKS(20));
    if (!VtAudio::initMic()) {
        LOG_ERROR("vtPlay: mic re-init failed after playback; record disabled until reboot");
    }

    LOG_INFO("vtPlay: done, %u frames decoded", (unsigned)frame_count);
    playback_done = true;
    playback_task = nullptr;
    vTaskDelete(NULL);
}

void VoicetasticModule::encoderTaskBody()
{
    using namespace voicetastic;
    // We're on the dedicated encoder task with 32 KB stack.
    int16_t pcm[VT_PCM_SAMPLES_PER_FRAME];
    uint8_t bits[16];
    const int wanted = VtAudio::bytesPerCodec2Frame();
    uint32_t frame_count = 0;

    while (rec_pcm_file && rec_pcm_file.available() >= (int)VT_PCM_BYTES_PER_FRAME) {
        const size_t got = rec_pcm_file.read((uint8_t *)pcm, VT_PCM_BYTES_PER_FRAME);
        if (got < VT_PCM_BYTES_PER_FRAME) break;

        VtAudio::encodeFrame(pcm, bits);
        encoder_result_audio.insert(encoder_result_audio.end(), bits, bits + wanted);
        frame_count++;
        if ((frame_count % 25) == 0) {
            LOG_DEBUG("vtEncode: %u/%u frames (%u bytes)",
                      (unsigned)frame_count, (unsigned)enc_frames_total,
                      (unsigned)encoder_result_audio.size());
        }
        // Polite yield so LVGL + other tasks get slots even though we're at
        // priority 1 on what should be a co-scheduled core.
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    if (rec_pcm_file) rec_pcm_file.close();
    if (FSCom.exists(VT_PCM_PATH)) FSCom.remove(VT_PCM_PATH);
    VtAudio::deinitEncoder();
    LOG_INFO("vtEncode: complete, %u bytes produced", (unsigned)encoder_result_audio.size());
    encoder_done = true;       // signal loop task
    encoder_task = nullptr;
    vTaskDelete(NULL);          // never returns
}

void VoicetasticModule::recordFrame()
{
    using namespace voicetastic;
    static int16_t pcm[VT_PCM_SAMPLES_PER_FRAME];
    static uint32_t rec_frame_count = 0;
    const size_t got = VtAudio::readPcm(pcm, VT_PCM_SAMPLES_PER_FRAME, 30);
    if ((int)got >= VT_PCM_SAMPLES_PER_FRAME && rec_pcm_file) {
        rec_pcm_file.write((const uint8_t *)pcm, VT_PCM_BYTES_PER_FRAME);
        rec_frame_count++;
        // Magnitude heartbeat every ~1 s: if pmax stays at 0 or a tiny single-
        // digit value, the mic is producing silence and the captured audio
        // will decode to nothing. A live mic should report pmax in the hundreds
        // for ambient noise, thousands for speech.
        if ((rec_frame_count % 25) == 0) {
            int16_t pmax = 0;
            for (int i = 0; i < VT_PCM_SAMPLES_PER_FRAME; i++) {
                const int16_t a = pcm[i] < 0 ? (int16_t)-pcm[i] : pcm[i];
                if (a > pmax) pmax = a;
            }
            LOG_DEBUG("vtRec: %u frames, pcm |max|=%d", (unsigned)rec_frame_count, (int)pmax);
        }
    }
    if ((millis() - rec_started_ms) >= rec_duration_ms) {
        stopRecording();
    }
}

void VoicetasticModule::sendOneChunk()
{
    using namespace voicetastic;
    const uint16_t walked = tx_next_chunk;
    const bool is_data = walked < tx_msg.total_data;
    const uint8_t  idx = is_data ? (uint8_t)walked : (uint8_t)(walked - tx_msg.total_data);
    const bool is_last_in_stream =
        (uint16_t)(walked + 1) == (uint16_t)tx_msg.total_data + (uint16_t)tx_msg.parity_count;
    txSendShard(is_data, idx, is_last_in_stream);
}

void VoicetasticModule::sendDataRetransmit(uint8_t data_idx)
{
    using namespace voicetastic;
    if (data_idx >= tx_msg.total_data) return;
    // NACK-driven retransmits ride the same wire path; we leave last_in_stream
    // clear since the original linear walk's terminal frame already carried it.
    txSendShard(/*is_data=*/true, data_idx, /*last_in_stream=*/false);
}

void VoicetasticModule::txSendShard(bool is_data, uint8_t idx, bool last_in_stream)
{
    using namespace voicetastic;

    // Look up the channel PSK (expanded form) only to feed the keyed-MAC.
    // Confidentiality is left to Meshtastic's per-packet AES-CTR layer — we
    // no longer wrap the body in the AES-GCM envelope (spec §7), saving the
    // 28 bytes/frame of nonce+tag and one HKDF derivation per frame. The
    // receive path still accepts encrypted=1 frames from peers that emit
    // them; we just never set the bit ourselves.
    uint8_t psk_buf[32] = {0};
    size_t  psk_len = 0;
    resolveChannelPsk(tx_channel, psk_buf, psk_len);
    const bool use_keyed_mac = mac_keyed_tx_enabled && psk_len > 0;
    const uint8_t *mac_key  = use_keyed_mac ? psk_buf : nullptr;
    const size_t   mac_klen = use_keyed_mac ? psk_len : 0;

    VtHeader h{};
    h.version        = PROTOCOL_VERSION;
    h.packet_type    = is_data ? PacketType::DATA : PacketType::PARITY;
    h.encrypted      = false;
    h.last_in_stream = last_in_stream;
    h.mac_keyed      = false; // overwritten by encodeHeader based on the key arg
    h.message_id     = tx_msg.message_id;
    h.codec          = tx_msg.codec;
    h.codec_param    = tx_msg.codec_param;
    h.stream_seq     = tx_msg.stream_seq;
    h.chunk_index    = idx;
    h.total_data     = tx_msg.total_data;
    h.parity_count   = tx_msg.parity_count;

    uint8_t header_buf[HEADER_SIZE] = {0};
    if (encodeHeader(h, header_buf, mac_key, mac_klen) != HEADER_SIZE) {
        LOG_ERROR("Voicetastic: header encode failed (idx=%u)", (unsigned)idx);
        return;
    }

    // Body: trimmed for final DATA, full for all others.
    const std::vector<uint8_t> &src = is_data ? tx_msg.data[idx] : tx_msg.parity[idx];
    const size_t body_len =
        (is_data && idx == tx_msg.total_data - 1) ? tx_msg.last_data_real_size : (size_t)tx_msg.chunk_size;

    meshtastic_MeshPacket *p = allocDataPacket();
    p->to = tx_to;
    p->channel = tx_channel;
    p->want_ack = false;
    p->decoded.want_response = false;
    p->priority = meshtastic_MeshPacket_Priority_DEFAULT;

    const size_t total = HEADER_SIZE + body_len;
    if (total > sizeof(p->decoded.payload.bytes)) {
        LOG_ERROR("Voicetastic: chunk too large (%u)", (unsigned)total);
        p->decoded.payload.size = 0;
        return;
    }
    memcpy(p->decoded.payload.bytes, header_buf, HEADER_SIZE);
    memcpy(p->decoded.payload.bytes + HEADER_SIZE, src.data(), body_len);
    p->decoded.payload.size = total;

    LOG_DEBUG("Voicetastic: tx mid=%08x %s ci=%u/%u body=%u",
              (unsigned)h.message_id, is_data ? "D" : "P",
              (unsigned)h.chunk_index, (unsigned)tx_msg.total_data, (unsigned)body_len);
    service->sendToMesh(p);
}

void VoicetasticModule::sendNack(const voicetastic::VtAssembler::PendingNack &nk)
{
    using namespace voicetastic;

    VtHeader h{};
    h.version        = PROTOCOL_VERSION;
    h.packet_type    = PacketType::NACK;
    h.encrypted      = false;     // spec §3.4: NACKs MUST NOT use the GCM envelope
    h.last_in_stream = false;
    h.mac_keyed      = false;
    h.message_id     = nk.message_id;
    h.codec          = nk.codec;
    h.codec_param    = nk.codec_param;
    h.stream_seq     = nk.stream_seq;
    h.chunk_index    = 0;         // spec §3.4
    h.total_data     = nk.total_data;
    h.parity_count   = nk.parity_count;

    uint8_t psk_buf[32] = {0};
    size_t  psk_len = 0;
    resolveChannelPsk(nk.channel, psk_buf, psk_len);
    const bool use_keyed_mac = mac_keyed_tx_enabled && psk_len > 0;
    const uint8_t *mac_key  = use_keyed_mac ? psk_buf : nullptr;
    const size_t   mac_klen = use_keyed_mac ? psk_len : 0;

    uint8_t header_buf[HEADER_SIZE] = {0};
    if (encodeHeader(h, header_buf, mac_key, mac_klen) != HEADER_SIZE) {
        LOG_ERROR("Voicetastic: NACK header encode failed (mid=%08x)", (unsigned)nk.message_id);
        return;
    }

    // Body: spec §3.4.
    uint8_t body_buf[2 + ((255u + 7u) / 8u)] = {0};
    // Convert vector<bool> to a plain bool array for encodeNackBody.
    bool missing_buf[255] = {false};
    for (uint8_t i = 0; i < nk.total_data; ++i) missing_buf[i] = nk.missing[i];
    const size_t body_len = encodeNackBody(nk.total_data, missing_buf, nk.give_up,
                                           body_buf, sizeof(body_buf));
    if (body_len == 0) {
        LOG_ERROR("Voicetastic: NACK body encode failed (mid=%08x total=%u)",
                  (unsigned)nk.message_id, (unsigned)nk.total_data);
        return;
    }

    meshtastic_MeshPacket *p = allocDataPacket();
    p->to = nk.from;          // back to the original sender
    p->channel = nk.channel;
    p->want_ack = false;      // spec §3.4
    p->decoded.want_response = false;
    p->priority = meshtastic_MeshPacket_Priority_DEFAULT;

    const size_t total = HEADER_SIZE + body_len;
    if (total > sizeof(p->decoded.payload.bytes)) {
        p->decoded.payload.size = 0;
        return;
    }
    memcpy(p->decoded.payload.bytes, header_buf, HEADER_SIZE);
    memcpy(p->decoded.payload.bytes + HEADER_SIZE, body_buf, body_len);
    p->decoded.payload.size = total;

    // Count missing for the log line.
    unsigned nmiss = 0;
    for (uint8_t i = 0; i < nk.total_data; ++i) if (nk.missing[i]) nmiss++;
    LOG_INFO("Voicetastic: NACK mid=%08x to=0x%08x missing=%u/%u%s",
             (unsigned)nk.message_id, (unsigned)nk.from, nmiss, (unsigned)nk.total_data,
             nk.give_up ? " give_up" : "");
    service->sendToMesh(p);
}

ProcessMessage VoicetasticModule::handleReceived(const meshtastic_MeshPacket &mp)
{
    using namespace voicetastic;
    const auto &p = mp.decoded;

    if (p.payload.size < HEADER_SIZE) {
        LOG_DEBUG("Voicetastic: drop short PRIVATE_APP packet (%u bytes)", (unsigned)p.payload.size);
        return ProcessMessage::CONTINUE;
    }

    // Resolve the inbound channel's PSK so we can verify keyed MACs. Frames
    // advertising mac_keyed=1 on a channel where we have no PSK will be rejected
    // by decodeHeader (no way to validate authenticity).
    uint8_t rx_psk[32] = {0};
    size_t  rx_psk_len = 0;
    resolveChannelPsk(mp.channel, rx_psk, rx_psk_len);
    const uint8_t *rx_key  = (rx_psk_len > 0) ? rx_psk : nullptr;
    const size_t   rx_klen = rx_psk_len;

    VtHeader h{};
    if (!decodeHeader(p.payload.bytes, h, rx_key, rx_klen)) {
        LOG_DEBUG("Voicetastic: header reject (ver=0x%02x size=%u ch=%u keyed=%u)",
                  (unsigned)p.payload.bytes[0], (unsigned)p.payload.size,
                  (unsigned)mp.channel, (unsigned)((p.payload.bytes[1] & MASK_MAC_KEYED) != 0));
        return ProcessMessage::CONTINUE;
    }

    const size_t body_len = p.payload.size - HEADER_SIZE;
    char prefix[40];
    snprintf(prefix, sizeof(prefix), "rx from=0x%08x body=%u", (unsigned)mp.from, (unsigned)body_len);
    logHeader(prefix, h);

    // Inbound NACK: route to the active TX state. We don't carry per-channel TX
    // state — there's only one outbound message in flight at a time — so we
    // match purely by message_id.
    if (h.packet_type == PacketType::NACK) {
        if (!tx_active || tx_msg.message_id != h.message_id) {
            LOG_DEBUG("Voicetastic: NACK for mid=%08x ignored (no matching TX)",
                      (unsigned)h.message_id);
            return ProcessMessage::CONTINUE;
        }
        if (h.total_data != tx_msg.total_data) {
            LOG_WARN("Voicetastic: NACK mid=%08x rejected (total_data drift %u vs %u)",
                     (unsigned)h.message_id, (unsigned)h.total_data, (unsigned)tx_msg.total_data);
            return ProcessMessage::CONTINUE;
        }
        bool missing_buf[255] = {false};
        bool give_up = false;
        if (!decodeNackBody(p.payload.bytes + HEADER_SIZE, body_len,
                            h.total_data, missing_buf, give_up)) {
            LOG_WARN("Voicetastic: NACK body decode failed (mid=%08x body=%u)",
                     (unsigned)h.message_id, (unsigned)body_len);
            return ProcessMessage::CONTINUE;
        }
        if (give_up) {
            LOG_INFO("Voicetastic: NACK give_up mid=%08x — aborting TX",
                     (unsigned)h.message_id);
            tx_active = false;
            tx_retransmit_idx.clear();
            tx_msg.data.clear();
            tx_msg.parity.clear();
            return ProcessMessage::CONTINUE;
        }
        // Append missing data chunks to the retransmit queue, skipping ones
        // already pending so a chatty NACK loop doesn't compound airtime.
        unsigned added = 0;
        for (uint8_t i = 0; i < h.total_data; ++i) {
            if (!missing_buf[i]) continue;
            bool already = false;
            for (uint8_t q : tx_retransmit_idx) {
                if (q == i) { already = true; break; }
            }
            if (!already) { tx_retransmit_idx.push_back(i); added++; }
        }
        // Extend the linger window to keep tx_msg state alive long enough to
        // service the new retransmits.
        tx_linger_until_ms = millis() + TX_NACK_LINGER_MS;
        LOG_INFO("Voicetastic: NACK mid=%08x queued %u retransmits (queue=%u)",
                 (unsigned)h.message_id, added, (unsigned)tx_retransmit_idx.size());
        return ProcessMessage::CONTINUE;
    }

    // If the sender wrapped the body in an AES-GCM envelope (spec §7),
    // decrypt it before handing off to the assembler. We keep the plaintext
    // in a stack buffer sized for one frame body.
    const uint8_t *body_ptr = p.payload.bytes + HEADER_SIZE;
    size_t         body_use_len = body_len;
    uint8_t        decrypted_buf[MAX_PLAINTEXT_BODY];
    if (h.encrypted) {
        if (rx_psk_len == 0) {
            LOG_DEBUG("Voicetastic: dropping encrypted frame mid=%08x on PSK-less channel",
                      (unsigned)h.message_id);
            return ProcessMessage::CONTINUE;
        }
        if (body_len < GCM_NONCE_LEN + GCM_TAG_LEN) {
            LOG_WARN("Voicetastic: encrypted body too short (%u)", (unsigned)body_len);
            return ProcessMessage::CONTINUE;
        }
        const size_t ct_len = body_len - GCM_NONCE_LEN - GCM_TAG_LEN;
        if (ct_len > sizeof(decrypted_buf)) {
            LOG_WARN("Voicetastic: encrypted ciphertext too large (%u)", (unsigned)ct_len);
            return ProcessMessage::CONTINUE;
        }
        uint8_t key32[GCM_KEY_LEN];
        if (!deriveEnvelopeKey(rx_psk, rx_psk_len, h.message_id, mp.from, key32)) {
            LOG_WARN("Voicetastic: HKDF failed for mid=%08x", (unsigned)h.message_id);
            return ProcessMessage::CONTINUE;
        }
        // AAD = header[0..12] per spec §7.
        if (!gcmDecrypt(key32,
                        /*nonce=*/ p.payload.bytes + HEADER_SIZE,
                        /*aad=*/ p.payload.bytes, HEADER_SIZE - MAC_TAG_SIZE,
                        /*ct=*/ p.payload.bytes + HEADER_SIZE + GCM_NONCE_LEN, ct_len,
                        /*tag=*/ p.payload.bytes + HEADER_SIZE + GCM_NONCE_LEN + ct_len,
                        decrypted_buf)) {
            LOG_WARN("Voicetastic: GCM auth failed mid=%08x ci=%u",
                     (unsigned)h.message_id, (unsigned)h.chunk_index);
            return ProcessMessage::CONTINUE;
        }
        body_ptr = decrypted_buf;
        body_use_len = ct_len;
    }

    // Hand DATA / PARITY frames to the per-(from, message_id) assembler. It
    // deals with shard storage, chunk_size inference (spec §4), FEC
    // reconstruction, and queuing complete messages for playback (Phase 6).
    if (assembler.acceptFrame(mp.from, mp.to, mp.channel, h,
                              body_ptr, body_use_len)) {
        // A new message was just completed. Phase 6 will drain the assembler's
        // complete-queue into the playback engine; for now we just leave it
        // sitting there so anyone polling popComplete() can pick it up.
    }
    return ProcessMessage::CONTINUE;
}

int32_t VoicetasticModule::runOnce()
{
    using namespace voicetastic;
    const uint32_t now = millis();

    // Time out stuck inbound assemblies (absolute deadline) and check for
    // chunks ready to NACK on the quiet-window timer.
    assembler.tick(now);
    {
        VtAssembler::PendingNack nk;
        if (assembler.pollPendingNack(nk, now)) {
            sendNack(nk);
        }
    }

    // Playback lifecycle. If a worker task is running, wait for it; on
    // completion, clear state. Drain new assembler completions into our own
    // pending_play_queue but do NOT auto-play -- the chat-screen mini-player
    // calls playNextPending() explicitly when the user taps play.
    if (playing && playback_done) {
        playing = false;
        playback_done = false;
        playback_stop_requested = false;
        playback_msg.audio.clear();
        playback_msg.audio.shrink_to_fit();
        play_total_ms = 0;
        play_started_ms = 0;
        play_from_node = 0;
        play_message_id = 0;
        LOG_INFO("Voicetastic: playback complete");
    }
    ReceivedVoiceMessage rxmsg;
    while (assembler.popComplete(rxmsg)) {
        LOG_INFO("Voicetastic: voice from 0x%08x queued for play (%u bytes)",
                 (unsigned)rxmsg.from, (unsigned)rxmsg.audio.size());
        pending_play_queue.push_back(std::move(rxmsg));
        // Cap the inbox so a flood of voice messages doesn't OOM. ~16 entries
        // at ~5 KB/30 s gives a worst-case 80 KB resident, comfortably inside
        // our PSRAM/free-heap budget.
        while (pending_play_queue.size() > kMaxInbox) {
            pending_play_queue.erase(pending_play_queue.begin());
        }
    }

#ifdef VOICETASTIC_BOOT_MIC_TEST
    // One-shot boot test recording: ten seconds after boot, ask the codec2
    // task to capture ~3 s and broadcast it. Opt-in via build flag; useful
    // for verifying the mic + codec2 + TX path against voicetastic-desktop on
    // a tethered radio when no manual trigger is wired yet.
    if (!boot_test_sent && (now - boot_ms) > 10000) {
        boot_test_sent = true;
        if (!startRecording(3000)) {
            LOG_ERROR("Voicetastic: boot record request rejected");
        }
    }
#else
    (void)boot_test_sent; (void)boot_ms;
#endif

    // SD-buffered recording state machine.
    //   eRecRecording: read PCM from mic, write to SD. Cheap per-tick work,
    //                  schedule back fast (5 ms).
    //   eRecEncoding:  read one PCM frame from SD, run codec2_encode, append
    //                  to pending_audio. codec2_encode is heavy on the loop
    //                  task stack, so we pace at 50 ms between calls so LVGL
    //                  + the rest of the firmware get plenty of slots.
    if (rec_state == eRecRecording) {
        recordFrame();
        return 5;
    }
    if (rec_state == eRecEncoding) {
        // Encoder worker is running on its own task; just poll its done flag.
        if (encoder_done) {
            pending_audio = std::move(encoder_result_audio);
            encoder_result_audio.clear();
            encoder_result_audio.shrink_to_fit();
            encoder_done = false;
            rec_state = eRecIdle;
            LOG_INFO("Voicetastic: encoding done, %u bytes ready; ENTER to send",
                     (unsigned)pending_audio.size());
        }
        return 100;
    }

    if (!tx_active) return 500;

    const uint16_t total_chunks = (uint16_t)tx_msg.total_data + (uint16_t)tx_msg.parity_count;
    const bool linear_walk_done = (tx_next_chunk >= total_chunks);

    // Done? Linear walk complete AND no retransmits queued AND the NACK linger
    // window has elapsed → tear down TX state.
    if (linear_walk_done && tx_retransmit_idx.empty()) {
        if (tx_linger_until_ms == 0) {
            // Just finished the linear walk; start the linger so we can still
            // answer late NACKs (spec §8: NACK loop).
            tx_linger_until_ms = now + TX_NACK_LINGER_MS;
            LOG_INFO("Voicetastic: TX walk complete mid=%08x, linger %u ms for NACKs",
                     (unsigned)tx_msg.message_id, (unsigned)TX_NACK_LINGER_MS);
        } else if ((int32_t)(now - tx_linger_until_ms) >= 0) {
            LOG_INFO("Voicetastic: TX teardown mid=%08x (%u chunks)",
                     (unsigned)tx_msg.message_id, (unsigned)total_chunks);
            tx_active = false;
            tx_linger_until_ms = 0;
            tx_msg.data.clear(); tx_msg.parity.clear();
            return 500;
        }
        // Still inside linger window; idle but ready to react to NACKs.
        return 100;
    }

    // Pacing window not yet elapsed?
    if ((int32_t)(now - tx_paced_until_ms) < 0) {
        return (int32_t)(tx_paced_until_ms - now);
    }

    // Firmware-queue backpressure (spec §2.2): pause while < 3 free slots.
    meshtastic_QueueStatus qs = router->getQueueStatus();
    if (qs.free <= 2) {
        return 100;
    }

    // Retransmits ride ahead of the linear walk so NACKed chunks reach the
    // receiver before we finish parity and tear down.
    if (!tx_retransmit_idx.empty()) {
        const uint8_t idx = tx_retransmit_idx.front();
        tx_retransmit_idx.erase(tx_retransmit_idx.begin());
        sendDataRetransmit(idx);
    } else {
        sendOneChunk();
        tx_next_chunk++;
    }
    const auto preset = (meshtastic_Config_LoRaConfig_ModemPreset)config.lora.modem_preset;
    tx_paced_until_ms = now + pacingMsForPreset(preset);

    return 10; // re-enter quickly; pacing/backpressure gates above
}

#endif // ARCH_ESP32 && !MESHTASTIC_EXCLUDE_VOICETASTIC
