#pragma once

#include "configuration.h"

#if defined(ARCH_ESP32) && !MESHTASTIC_EXCLUDE_VOICETASTIC && HAS_TFT

#include "comms/PacketClient.h"

// Bridge between the meshtastic-device-ui chat screen and the firmware-side
// VoicetasticModule. Inherits PacketClient's normal radio plumbing and adds
// implementations of the voice* hooks declared on IClientBase.
class VoicetasticPacketClient : public PacketClient
{
  public:
    VoicetasticPacketClient() = default;

    bool      voiceRecordStart(uint32_t to_nodenum, uint8_t channel,
                               uint32_t max_duration_ms) override;
    void      voiceRecordStop() override;
    bool      voiceRecordSend() override;
    void      voiceRecordCancel() override;
    VoiceState voiceRecordState() const override;
    uint32_t  voiceRecordElapsedMs() const override;

    // Mini-player.
    size_t    voicePlayPendingCount() const override;
    bool      voicePlayNext() override;
    bool      voicePlayByMessageId(uint32_t message_id) override;
    void      voicePlayStop() override;
    bool      voicePlayIsPlaying() const override;
    uint32_t  voicePlayElapsedMs() const override;
    uint32_t  voicePlayTotalMs() const override;
    uint32_t  voicePlayFromNode() const override;
    uint32_t  voicePlayMessageId() const override;
    bool      voicePlayPeek(size_t index, uint32_t &from, uint32_t &message_id,
                            uint32_t &approx_duration_ms) const override;

  private:
    // The chat screen calls voiceRecordStart() with the currently-selected
    // peer/channel; we cache them so voiceRecordSend() knows where to send.
    uint32_t pending_to = 0xFFFFFFFFu;
    uint8_t  pending_channel = 0;
};

#endif // ARCH_ESP32 && !MESHTASTIC_EXCLUDE_VOICETASTIC && HAS_TFT
