#pragma once

#include "configuration.h"

#if defined(ARCH_ESP32) && !MESHTASTIC_EXCLUDE_VOICETASTIC

#include <stdint.h>
#include <stddef.h>

namespace voicetastic {

// Mic capture for the LilyGo T-Deck (ES7210 4-channel ADC -> I2S).
//
// The ES7210 only supports 11/16/22/32/44/48 kHz, so we capture at 16 kHz
// and decimate 2:1 to the 8 kHz that Codec2 expects. Decimation is a simple
// averaging pair-down (acceptable for narrowband speech).
//
// Hardware notes (variants/esp32s3/t-deck/variant.h):
//   - I2C SDA=18, SCL=8 -> ES7210 register interface
//   - I2S SCK=47, DIN=14, LRCK=21, MCLK=48 (ES7210 in I2S slave mode)
//   - KB_POWERON (GPIO 10) shares the 3.3 V rail with the audio subsystem;
//     main.cpp drives it HIGH at boot, so the codec is powered by the time
//     setupModules() runs.
//   - GPIO 21 is shared with DAC_I2S_MCLK; capture and playback are mutually
//     exclusive at the hardware level (matches our half-duplex model).
class VtAudio {
public:
    static constexpr int  CAPTURE_RATE_HZ = 16000;     // ES7210 input rate
    static constexpr int  OUTPUT_RATE_HZ  = 8000;      // post-decimation rate, fed to Codec2
    static constexpr int  DECIMATION      = CAPTURE_RATE_HZ / OUTPUT_RATE_HZ;

    // Initialize ES7210 over I2C and the I2S RX driver. Idempotent; subsequent
    // calls after success are no-ops. Returns true on success.
    static bool initMic();

    // Has initMic() succeeded?
    static bool isMicReady();

    // Read up to `max_samples` int16 PCM samples at OUTPUT_RATE_HZ (8 kHz mono)
    // into `out`. Blocks up to `timeout_ms`. Returns the number of samples
    // actually written. May be 0 on timeout.
    //
    // Internally reads 2 * max_samples 16 kHz samples from I2S and averages
    // adjacent pairs.
    static size_t readPcm(int16_t *out, size_t max_samples, uint32_t timeout_ms = 50);

    // Tear down I2S driver and codec. Call before switching to playback.
    static void deinitMic();
};

} // namespace voicetastic

#endif // ARCH_ESP32 && !MESHTASTIC_EXCLUDE_VOICETASTIC
