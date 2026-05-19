#include "VoicetasticModule.h"

#if defined(ARCH_ESP32) && !MESHTASTIC_EXCLUDE_VOICETASTIC

#include "MeshService.h"
#include "NodeDB.h"
#include "RadioInterface.h"
#include "Router.h"
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
                                        NodeNum to)
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

bool VoicetasticModule::sendPending(NodeNum to, uint8_t /*channel*/)
{
    using namespace voicetastic;
    if (pending_audio.empty()) return false;
    if (tx_active) { LOG_WARN("Voicetastic: TX busy; cannot send pending audio yet"); return false; }

    std::vector<uint8_t> audio = std::move(pending_audio);
    pending_audio.clear();
    pending_audio.shrink_to_fit();
    if (audio.empty()) return false;
    return enqueueOutbound(audio.data(), audio.size(),
                           CodecId::CODEC2, (uint8_t)codec2_mode, to);
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

bool VoicetasticModule::startPlayback(const voicetastic::ReceivedVoiceMessage &msg)
{
    using namespace voicetastic;
    if (playing) return false;
    if (rec_state != eRecIdle) return false;     // can't play while recording/encoding
    if (msg.audio.empty()) return false;

    playback_msg = msg; // copy in (vector copy)
    playback_done = false;

    const BaseType_t rc = xTaskCreate(&VoicetasticModule::playbackTaskTrampoline,
                                      "vtPlay", 32768, this, 1, &playback_task);
    if (rc != pdPASS) {
        LOG_ERROR("Voicetastic: failed to create playback task (rc=%d)", (int)rc);
        playback_task = nullptr;
        playback_msg.audio.clear();
        return false;
    }
    playing = true;
    LOG_INFO("Voicetastic: playback started mid=%08x (%u bytes, codec=%u/%u)",
             (unsigned)msg.message_id, (unsigned)msg.audio.size(),
             (unsigned)msg.codec, (unsigned)msg.codec_param);
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

    while (consumed + bytes_per_frame <= total) {
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
    // KNOWN LIMITATION: we don't re-init the mic here. GPIO 21 is shared
    // between ES7210_LRCK and DAC_I2S_MCLK; the legacy i2s_legacy driver
    // doesn't fully reset the pin's drive direction on i2s_driver_uninstall,
    // so a subsequent i2s_driver_install on the mic's port (I2S_NUM_1) was
    // failing. Recording is unavailable until the device reboots after a
    // playback. Proper fix is to either pinMode(GPIO 21, INPUT) here or move
    // to the new i2s_std driver which has explicit per-channel teardown.

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
    const uint16_t idx = tx_next_chunk;
    const bool is_data = idx < tx_msg.total_data;
    const uint16_t parity_idx = is_data ? 0 : (idx - tx_msg.total_data);
    const bool is_last_in_stream =
        (uint16_t)(idx + 1) == (uint16_t)tx_msg.total_data + (uint16_t)tx_msg.parity_count;

    VtHeader h{};
    h.version        = PROTOCOL_VERSION;
    h.packet_type    = is_data ? PacketType::DATA : PacketType::PARITY;
    h.encrypted      = false;
    h.last_in_stream = is_last_in_stream;
    h.mac_keyed      = false;
    h.message_id     = tx_msg.message_id;
    h.codec          = tx_msg.codec;
    h.codec_param    = tx_msg.codec_param;
    h.stream_seq     = tx_msg.stream_seq;
    h.chunk_index    = is_data ? (uint8_t)idx : (uint8_t)parity_idx;
    h.total_data     = tx_msg.total_data;
    h.parity_count   = tx_msg.parity_count;

    uint8_t header_buf[HEADER_SIZE] = {0};
    if (encodeHeader(h, header_buf) != HEADER_SIZE) {
        LOG_ERROR("Voicetastic: header encode failed (idx=%u)", (unsigned)idx);
        return;
    }

    // Determine body size: trimmed for final DATA, full for all others.
    const std::vector<uint8_t> &src = is_data ? tx_msg.data[idx] : tx_msg.parity[parity_idx];
    const size_t body_len =
        (is_data && idx == tx_msg.total_data - 1) ? tx_msg.last_data_real_size : (size_t)tx_msg.chunk_size;

    meshtastic_MeshPacket *p = allocDataPacket();
    p->to = tx_to;
    p->want_ack = false; // no NACK loop in this build
    p->decoded.want_response = false;
    p->priority = meshtastic_MeshPacket_Priority_DEFAULT;

    const size_t total = HEADER_SIZE + body_len;
    if (total > sizeof(p->decoded.payload.bytes)) {
        LOG_ERROR("Voicetastic: chunk too large (%u)", (unsigned)total);
        // Allocated packet must be freed; in this codebase service->sendToMesh
        // takes ownership only on send. Fall through with size=0 to avoid leak.
        p->decoded.payload.size = 0;
    } else {
        memcpy(p->decoded.payload.bytes, header_buf, HEADER_SIZE);
        memcpy(p->decoded.payload.bytes + HEADER_SIZE, src.data(), body_len);
        p->decoded.payload.size = total;
    }

    LOG_DEBUG("Voicetastic: tx mid=%08x %s ci=%u/%u body=%u",
              (unsigned)h.message_id, is_data ? "D" : "P",
              (unsigned)h.chunk_index, (unsigned)tx_msg.total_data, (unsigned)body_len);
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

    VtHeader h{};
    if (!decodeHeader(p.payload.bytes, h)) {
        LOG_DEBUG("Voicetastic: header reject (ver=0x%02x size=%u)", (unsigned)p.payload.bytes[0],
                  (unsigned)p.payload.size);
        return ProcessMessage::CONTINUE;
    }

    const size_t body_len = p.payload.size - HEADER_SIZE;
    char prefix[40];
    snprintf(prefix, sizeof(prefix), "rx from=0x%08x body=%u", (unsigned)mp.from, (unsigned)body_len);
    logHeader(prefix, h);

    // Hand the frame to the per-(from, message_id) assembler. It deals with
    // shard storage, chunk_size inference (spec §4), FEC reconstruction, and
    // queuing complete messages for playback (Phase 6).
    if (assembler.acceptFrame(mp.from, h, p.payload.bytes + HEADER_SIZE, body_len)) {
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

    // Time out stuck inbound assemblies (no NACK loop in this build, so we
    // can't recover them; drop them so they don't leak.)
    assembler.tick(now);

    // Playback lifecycle. If a worker task is running, just wait for it; on
    // completion, clear state. If idle (no recording, no playback, no TX),
    // drain the next complete message from the assembler and auto-play it.
    if (playing && playback_done) {
        playing = false;
        playback_done = false;
        playback_msg.audio.clear();
        playback_msg.audio.shrink_to_fit();
        LOG_INFO("Voicetastic: playback complete");
    }
    if (!playing && rec_state == eRecIdle && !tx_active) {
        ReceivedVoiceMessage rxmsg;
        if (assembler.popComplete(rxmsg)) {
            startPlayback(rxmsg);
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

    // Done?
    const uint16_t total_chunks = (uint16_t)tx_msg.total_data + (uint16_t)tx_msg.parity_count;
    if (tx_next_chunk >= total_chunks) {
        LOG_INFO("Voicetastic: TX complete mid=%08x (%u chunks)",
                 (unsigned)tx_msg.message_id, (unsigned)total_chunks);
        tx_active = false;
        // Release the shard buffers so we don't hold ~tens of KB indefinitely.
        tx_msg.data.clear(); tx_msg.parity.clear();
        return 500;
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

    sendOneChunk();
    tx_next_chunk++;
    const auto preset = (meshtastic_Config_LoRaConfig_ModemPreset)config.lora.modem_preset;
    tx_paced_until_ms = now + pacingMsForPreset(preset);

    return 10; // re-enter quickly; pacing/backpressure gates above
}

#endif // ARCH_ESP32 && !MESHTASTIC_EXCLUDE_VOICETASTIC
