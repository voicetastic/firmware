#include "VoicetasticPacketClient.h"

#if defined(ARCH_ESP32) && !MESHTASTIC_EXCLUDE_VOICETASTIC && HAS_TFT

#include "modules/esp32/voicetastic/VoicetasticModule.h"

bool VoicetasticPacketClient::voiceRecordStart(uint32_t to_nodenum, uint8_t channel,
                                               uint32_t max_duration_ms)
{
    if (voicetasticModule == nullptr) return false;
    pending_to = to_nodenum;
    pending_channel = channel;
    // The destination is stored locally; voicetasticModule uses it at send time.
    return voicetasticModule->startRecording(max_duration_ms, (NodeNum)to_nodenum);
}

void VoicetasticPacketClient::voiceRecordStop()
{
    if (voicetasticModule == nullptr) return;
    voicetasticModule->stopRecording();
}

bool VoicetasticPacketClient::voiceRecordSend()
{
    if (voicetasticModule == nullptr) return false;
    if (!voicetasticModule->hasPending()) return false;
    return voicetasticModule->sendPending((NodeNum)pending_to, pending_channel);
}

void VoicetasticPacketClient::voiceRecordCancel()
{
    if (voicetasticModule == nullptr) return;
    if (voicetasticModule->isRecording()) {
        voicetasticModule->stopRecording();
    }
    voicetasticModule->discardPending();
}

IClientBase::VoiceState VoicetasticPacketClient::voiceRecordState() const
{
    if (voicetasticModule == nullptr) return eVoiceIdle;
    if (voicetasticModule->isRecording()) return eVoiceRecording;
    if (voicetasticModule->hasPending())  return eVoiceArmed;
    return eVoiceIdle;
}

uint32_t VoicetasticPacketClient::voiceRecordElapsedMs() const
{
    if (voicetasticModule == nullptr) return 0;
    return voicetasticModule->recordElapsedMs();
}

uint8_t VoicetasticPacketClient::voiceGetCodec2Mode() const
{
    if (voicetasticModule == nullptr) return 5; // M_1200 default
    return (uint8_t)voicetasticModule->getCodec2Mode();
}

void VoicetasticPacketClient::voiceSetCodec2Mode(uint8_t mode)
{
    if (voicetasticModule == nullptr) return;
    if (mode > 5) return; // clamp to the valid Codec2Mode enum range
    voicetasticModule->setCodec2Mode((voicetastic::Codec2Mode)mode);
}

// ---------- Mini-player ----------

size_t VoicetasticPacketClient::voicePlayPendingCount() const
{
    if (voicetasticModule == nullptr) return 0;
    return voicetasticModule->pendingPlayCount();
}

bool VoicetasticPacketClient::voicePlayNext()
{
    if (voicetasticModule == nullptr) return false;
    return voicetasticModule->playNextPending();
}

bool VoicetasticPacketClient::voicePlayByMessageId(uint32_t message_id)
{
    if (voicetasticModule == nullptr) return false;
    return voicetasticModule->playByMessageId(message_id);
}

uint32_t VoicetasticPacketClient::voicePlayMessageId() const
{
    if (voicetasticModule == nullptr) return 0;
    return voicetasticModule->playbackMessageId();
}

void VoicetasticPacketClient::voicePlayStop()
{
    if (voicetasticModule == nullptr) return;
    voicetasticModule->stopPlayback();
}

bool VoicetasticPacketClient::voicePlayIsPlaying() const
{
    if (voicetasticModule == nullptr) return false;
    return voicetasticModule->isPlaying();
}

uint32_t VoicetasticPacketClient::voicePlayElapsedMs() const
{
    if (voicetasticModule == nullptr) return 0;
    return voicetasticModule->playbackElapsedMs();
}

uint32_t VoicetasticPacketClient::voicePlayTotalMs() const
{
    if (voicetasticModule == nullptr) return 0;
    return voicetasticModule->playbackTotalMs();
}

uint32_t VoicetasticPacketClient::voicePlayFromNode() const
{
    if (voicetasticModule == nullptr) return 0;
    return (uint32_t)voicetasticModule->playbackFromNode();
}

bool VoicetasticPacketClient::voicePlayPeek(size_t index, uint32_t &from, uint32_t &message_id,
                                            uint32_t &approx_duration_ms) const
{
    if (voicetasticModule == nullptr) {
        from = 0; message_id = 0; approx_duration_ms = 0;
        return false;
    }
    NodeNum nn = 0;
    const bool ok = voicetasticModule->peekPending(index, nn, message_id, approx_duration_ms);
    from = (uint32_t)nn;
    return ok;
}

bool VoicetasticPacketClient::voicePlayPeekFull(size_t index, uint32_t &from, uint32_t &to,
                                                uint8_t &channel, uint32_t &message_id,
                                                uint32_t &approx_duration_ms, bool &played) const
{
    if (voicetasticModule == nullptr) {
        from = 0; to = 0; channel = 0; message_id = 0; approx_duration_ms = 0; played = false;
        return false;
    }
    NodeNum from_nn = 0, to_nn = 0;
    const bool ok = voicetasticModule->peekPendingFull(index, from_nn, to_nn, channel,
                                                       message_id, approx_duration_ms, played);
    from = (uint32_t)from_nn;
    to = (uint32_t)to_nn;
    return ok;
}

#endif // ARCH_ESP32 && !MESHTASTIC_EXCLUDE_VOICETASTIC && HAS_TFT
