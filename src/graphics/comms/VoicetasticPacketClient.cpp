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

#endif // ARCH_ESP32 && !MESHTASTIC_EXCLUDE_VOICETASTIC && HAS_TFT
