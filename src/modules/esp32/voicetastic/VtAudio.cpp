#include "VtAudio.h"

#if defined(ARCH_ESP32) && !MESHTASTIC_EXCLUDE_VOICETASTIC

#include "audio_drivers/es7210.h"
#include "configuration.h"
#include <Wire.h>
#include <codec2.h>
#include <driver/i2s.h>
#include <stdlib.h>
#include <string.h>

namespace voicetastic {

namespace {

// Use I2S_NUM_1 so we don't collide with anything that might use I2S_NUM_0
// (AudioModule on SX1280 builds, for example). T-Deck only has one I2S codec
// chain we care about so this is essentially exclusive ownership.
constexpr i2s_port_t   kI2sPort = I2S_NUM_1;

bool s_initialized = false;

// Scratch buffer for one read of raw 16 kHz interleaved (2-channel) samples
// before decimation. Sized for ~40 ms of audio = 640 input samples per call.
constexpr size_t kScratchMax = 1280;
int16_t s_scratch[kScratchMax];

// Codec2 encoder state. NULL when closed.
struct CODEC2 *s_codec2 = nullptr;
Codec2Mode s_codec2_mode = Codec2Mode::M_1200;
int s_codec2_samples = 0;
int s_codec2_bytes   = 0;

// Codec2 decoder state (independent of encoder). NULL when closed.
struct CODEC2 *s_codec2_dec = nullptr;
Codec2Mode s_codec2_dec_mode = Codec2Mode::M_1200;
int s_codec2_dec_samples = 0;
int s_codec2_dec_bytes   = 0;

// I2S DAC state.
constexpr i2s_port_t kDacI2sPort = I2S_NUM_0;
bool s_dac_initialized = false;

} // namespace

bool VtAudio::isMicReady() { return s_initialized; }

bool VtAudio::initMic()
{
    if (s_initialized) return true;

    // Wire is brought up earlier in boot for the keyboard; calling begin()
    // again with the same pins is a no-op on arduino-esp32.
    Wire.begin(I2C_SDA, I2C_SCL);

    audio_hal_codec_config_t cfg = {};
    cfg.adc_input        = AUDIO_HAL_ADC_INPUT_ALL;
    cfg.codec_mode       = AUDIO_HAL_CODEC_MODE_ENCODE;
    cfg.i2s_iface.mode   = AUDIO_HAL_MODE_SLAVE;
    cfg.i2s_iface.fmt    = AUDIO_HAL_I2S_NORMAL;
    cfg.i2s_iface.samples = AUDIO_HAL_16K_SAMPLES;
    cfg.i2s_iface.bits   = AUDIO_HAL_BIT_LENGTH_16BITS;

    if (es7210_adc_init(&Wire, &cfg) != ESP_OK) {
        LOG_ERROR("VtAudio: es7210_adc_init failed");
        return false;
    }
    if (es7210_adc_config_i2s(cfg.codec_mode, &cfg.i2s_iface) != ESP_OK) {
        LOG_ERROR("VtAudio: es7210_adc_config_i2s failed");
        return false;
    }
    // ES7210 routes ADC1/ADC2 to SDOUT1 and ADC3/ADC4 to SDOUT2 (see
    // es7210_config_fmt: REG12=0x00). Only SDOUT1 is wired to the ESP32
    // (ES7210_DIN = GPIO 14), so even though the LilyGo example documents
    // the mic on MIC3+MIC4 with 37.5 dB gain there, those samples never
    // reach us — we're physically reading MIC1+MIC2 on the SDOUT1 pin.
    // Apply the same 37.5 dB gain across all four mic preamps so whichever
    // pair carries the actual mic ends up audible. The unused side just
    // amplifies its own noise floor by ~5 dB more, which Codec2 handles
    // fine since it's narrowband-speech-tuned.
    es7210_adc_set_gain_all((es7210_gain_value_t)GAIN_37_5DB);
    if (es7210_adc_ctrl_state(cfg.codec_mode, AUDIO_HAL_CTRL_START) != ESP_OK) {
        LOG_ERROR("VtAudio: es7210_adc_ctrl_state(START) failed");
        return false;
    }

    i2s_config_t i2s_cfg = {};
    i2s_cfg.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX);
    i2s_cfg.sample_rate = CAPTURE_RATE_HZ;
    i2s_cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
    i2s_cfg.channel_format = I2S_CHANNEL_FMT_ALL_LEFT;
    i2s_cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
    i2s_cfg.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
    i2s_cfg.dma_buf_count = 8;
    i2s_cfg.dma_buf_len = 64;
    i2s_cfg.use_apll = false;
    i2s_cfg.tx_desc_auto_clear = true;
    i2s_cfg.fixed_mclk = 0;
    i2s_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_256;
    i2s_cfg.bits_per_chan = I2S_BITS_PER_CHAN_16BIT;
    i2s_cfg.chan_mask = (i2s_channel_t)(I2S_TDM_ACTIVE_CH0 | I2S_TDM_ACTIVE_CH1);

    if (i2s_driver_install(kI2sPort, &i2s_cfg, 0, NULL) != ESP_OK) {
        LOG_ERROR("VtAudio: i2s_driver_install failed");
        return false;
    }

    i2s_pin_config_t pins = {};
    pins.bck_io_num   = ES7210_SCK;
    pins.ws_io_num    = ES7210_LRCK;
    pins.data_in_num  = ES7210_DIN;
    pins.data_out_num = I2S_PIN_NO_CHANGE;
    pins.mck_io_num   = ES7210_MCLK;

    if (i2s_set_pin(kI2sPort, &pins) != ESP_OK) {
        LOG_ERROR("VtAudio: i2s_set_pin failed");
        i2s_driver_uninstall(kI2sPort);
        return false;
    }
    i2s_zero_dma_buffer(kI2sPort);

    s_initialized = true;
    LOG_INFO("VtAudio: ES7210 initialized; capture %d Hz -> decimate to %d Hz",
             CAPTURE_RATE_HZ, OUTPUT_RATE_HZ);
    return true;
}

void VtAudio::deinitMic()
{
    if (!s_initialized) return;
    i2s_driver_uninstall(kI2sPort);
    es7210_adc_deinit();
    // GPIO 21 is shared between ES7210_LRCK and DAC_I2S_MCLK. The legacy I2S
    // driver leaves pins configured as outputs after uninstall, which causes
    // the next install on the same pin to fail. Force the shared pins back
    // to a neutral input state so the DAC (or a future re-init of the mic
    // itself) can grab them cleanly.
    pinMode(ES7210_SCK, INPUT);
    pinMode(ES7210_DIN, INPUT);
    pinMode(ES7210_LRCK, INPUT);   // == GPIO 21, shared with DAC_I2S_MCLK
    pinMode(ES7210_MCLK, INPUT);
    s_initialized = false;
}

size_t VtAudio::readPcm(int16_t *out, size_t max_samples, uint32_t timeout_ms)
{
    if (!s_initialized || out == nullptr || max_samples == 0) return 0;

    // We want `max_samples` 8 kHz mono samples. ES7210 delivers two-channel
    // interleaved at 16 kHz: 4 input samples per output sample (2 channels × 2
    // for decimation). Read CH0 only, decimate 2:1 by averaging adjacent pairs.
    const size_t want_input_samples = max_samples * DECIMATION * 2; // *2 for stereo interleave
    const size_t take = (want_input_samples < kScratchMax) ? want_input_samples : kScratchMax;

    size_t bytes_read = 0;
    if (i2s_read(kI2sPort, s_scratch, take * sizeof(int16_t), &bytes_read,
                 pdMS_TO_TICKS(timeout_ms)) != ESP_OK) {
        return 0;
    }
    const size_t samples_read = bytes_read / sizeof(int16_t);
    // Indices 0,2,4,... are CH0 (16 kHz). Decimate 2:1 by averaging
    // (samples_in[0]+samples_in[2])/2, (samples_in[4]+samples_in[6])/2, ...
    size_t produced = 0;
    for (size_t i = 0; i + 4 <= samples_read && produced < max_samples; i += 4, produced++) {
        int32_t a = s_scratch[i];
        int32_t b = s_scratch[i + 2];
        out[produced] = (int16_t)((a + b) / 2);
    }
    return produced;
}

bool VtAudio::initEncoder(Codec2Mode mode)
{
    if (s_codec2 != nullptr) {
        if (s_codec2_mode == mode) return true;  // already at the requested mode
        // Mode changed (e.g. user picked a different bitrate in the UI). Tear
        // down the live encoder before creating a fresh one — without this the
        // call silently kept the old mode and downstream samples/frame +
        // bytes/frame readers returned the wrong values.
        deinitEncoder();
    }
    s_codec2 = codec2_create((int)mode);
    if (s_codec2 == nullptr) {
        LOG_ERROR("VtAudio: codec2_create(mode=%d) failed", (int)mode);
        return false;
    }
    s_codec2_mode    = mode;
    s_codec2_samples = codec2_samples_per_frame(s_codec2);
    s_codec2_bytes   = (codec2_bits_per_frame(s_codec2) + 7) / 8;
    LOG_INFO("VtAudio: codec2 ready mode=%d samples/frame=%d bytes/frame=%d",
             (int)mode, s_codec2_samples, s_codec2_bytes);
    return true;
}

void VtAudio::deinitEncoder()
{
    if (s_codec2 == nullptr) return;
    codec2_destroy(s_codec2);
    s_codec2 = nullptr;
    s_codec2_samples = 0;
    s_codec2_bytes   = 0;
}

int VtAudio::samplesPerCodec2Frame() { return s_codec2_samples; }
int VtAudio::bytesPerCodec2Frame()   { return s_codec2_bytes; }

void VtAudio::encodeFrame(const int16_t *pcm, uint8_t *out_bytes)
{
    if (s_codec2 == nullptr) return;
    codec2_encode(s_codec2, out_bytes, const_cast<int16_t *>(pcm));
}

// ---------- Codec2 decoder ----------

bool VtAudio::initDecoder(Codec2Mode mode)
{
    if (s_codec2_dec != nullptr) {
        if (s_codec2_dec_mode == mode) return true; // already at the requested mode
        // Each inbound message carries its own codec_param on the wire, so the
        // playback worker can hit decode mode A on one message and mode B on
        // the next. Swap the decoder instance to match.
        deinitDecoder();
    }
    s_codec2_dec = codec2_create((int)mode);
    if (s_codec2_dec == nullptr) {
        LOG_ERROR("VtAudio: codec2_create(decoder, mode=%d) failed", (int)mode);
        return false;
    }
    s_codec2_dec_mode    = mode;
    s_codec2_dec_samples = codec2_samples_per_frame(s_codec2_dec);
    s_codec2_dec_bytes   = (codec2_bits_per_frame(s_codec2_dec) + 7) / 8;
    LOG_INFO("VtAudio: codec2 decoder ready mode=%d samples/frame=%d bytes/frame=%d",
             (int)mode, s_codec2_dec_samples, s_codec2_dec_bytes);
    return true;
}

void VtAudio::deinitDecoder()
{
    if (s_codec2_dec == nullptr) return;
    codec2_destroy(s_codec2_dec);
    s_codec2_dec = nullptr;
    s_codec2_dec_samples = 0;
    s_codec2_dec_bytes   = 0;
}

int VtAudio::samplesPerDecodedFrame() { return s_codec2_dec_samples; }
int VtAudio::bytesPerDecodedFrame()   { return s_codec2_dec_bytes; }

void VtAudio::decodeFrame(const uint8_t *bits, int16_t *pcm_out)
{
    if (s_codec2_dec == nullptr) return;
    codec2_decode(s_codec2_dec, pcm_out, bits);
}

// ---------- I2S DAC (MAX98357A) ----------

bool VtAudio::initDac()
{
    if (s_dac_initialized) return true;

    i2s_config_t cfg = {};
    cfg.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
    cfg.sample_rate = OUTPUT_RATE_HZ; // 8 kHz matches Codec2 output
    cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
    cfg.channel_format = I2S_CHANNEL_FMT_ONLY_LEFT;
    cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
    cfg.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
    cfg.dma_buf_count = 8;
    cfg.dma_buf_len = 64;
    cfg.use_apll = false;
    cfg.tx_desc_auto_clear = true;
    cfg.fixed_mclk = 0;
    cfg.mclk_multiple = I2S_MCLK_MULTIPLE_256;
    cfg.bits_per_chan = I2S_BITS_PER_CHAN_16BIT;

    if (i2s_driver_install(kDacI2sPort, &cfg, 0, NULL) != ESP_OK) {
        LOG_ERROR("VtAudio: dac i2s_driver_install failed");
        return false;
    }

    i2s_pin_config_t pins = {};
    pins.bck_io_num   = DAC_I2S_BCK;
    pins.ws_io_num    = DAC_I2S_WS;
    pins.data_out_num = DAC_I2S_DOUT;
    pins.data_in_num  = I2S_PIN_NO_CHANGE;
    pins.mck_io_num   = DAC_I2S_MCLK; // shared with ES7210_LRCK on GPIO 21

    if (i2s_set_pin(kDacI2sPort, &pins) != ESP_OK) {
        LOG_ERROR("VtAudio: dac i2s_set_pin failed");
        i2s_driver_uninstall(kDacI2sPort);
        return false;
    }
    i2s_zero_dma_buffer(kDacI2sPort);
    s_dac_initialized = true;
    LOG_INFO("VtAudio: DAC initialized at %d Hz mono", OUTPUT_RATE_HZ);
    return true;
}

void VtAudio::deinitDac()
{
    if (!s_dac_initialized) return;
    i2s_driver_uninstall(kDacI2sPort);
    // Symmetric pin reset (see deinitMic). The mic's I2S_NUM_1 install needs
    // to claim GPIO 21 for LRCK; the legacy I2S driver doesn't put pins back
    // into a fresh-input state on uninstall, so we do it explicitly.
    pinMode(DAC_I2S_BCK, INPUT);
    pinMode(DAC_I2S_WS, INPUT);
    pinMode(DAC_I2S_DOUT, INPUT);
    pinMode(DAC_I2S_MCLK, INPUT);  // == GPIO 21, shared with ES7210_LRCK
    s_dac_initialized = false;
}

size_t VtAudio::writePcm(const int16_t *pcm, size_t samples)
{
    if (!s_dac_initialized || pcm == nullptr || samples == 0) return 0;
    size_t bytes_written = 0;
    i2s_write(kDacI2sPort, pcm, samples * sizeof(int16_t), &bytes_written,
              pdMS_TO_TICKS(500));
    return bytes_written / sizeof(int16_t);
}

} // namespace voicetastic

#endif // ARCH_ESP32 && !MESHTASTIC_EXCLUDE_VOICETASTIC
