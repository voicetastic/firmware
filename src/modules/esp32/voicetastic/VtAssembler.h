#pragma once

#include "configuration.h"

#if defined(ARCH_ESP32) && !MESHTASTIC_EXCLUDE_VOICETASTIC

#include "VtProtocol.h"
#include "mesh/MeshTypes.h"
#include <stdint.h>
#include <vector>

namespace voicetastic {

// One reassembled voice message, ready for playback or hand-off.
struct ReceivedVoiceMessage {
    NodeNum   from;
    uint32_t  message_id;
    CodecId   codec;
    uint8_t   codec_param;
    uint8_t   total_data;
    uint8_t   received_data;       // data shards that arrived directly
    uint8_t   recovered_via_fec;   // data shards reconstructed by RS
    uint32_t  completed_ms;
    std::vector<uint8_t> audio;    // reassembled codec frame bytes (no container)
};

// Per-(from, message_id) reassembly state machine, scoped down from spec §9
// for our MVP: no NACK round-trips, no global blacklist, just enough to
// recover messages whose loss falls inside parity_count and to time out
// stuck assemblies.
class VtAssembler {
  public:
    VtAssembler() = default;

    // Accept a parsed v2 frame. Returns true iff the call finalized a new
    // message (popComplete() is now non-empty).
    bool acceptFrame(NodeNum from, const VtHeader &h, const uint8_t *body, size_t body_len);

    // Drop stuck assemblies older than the timeout (called from runOnce).
    void tick(uint32_t now_ms);

    // Pop the oldest completed message into `out`. Returns false if none.
    bool popComplete(ReceivedVoiceMessage &out);

    // Diagnostic counts.
    size_t inProgressCount() const;
    size_t completeQueueDepth() const { return complete_count; }

  private:
    static constexpr int  MAX_IN_PROGRESS = 4;        // per-sender * 1 sender in MVP
    static constexpr int  MAX_COMPLETE_QUEUE = 4;     // recent received messages
    static constexpr uint32_t TIMEOUT_MS = 30000;     // drop a stuck assembly after 30 s

    struct AssemblyState {
        bool       in_use = false;
        NodeNum    from;
        uint32_t   message_id;
        CodecId    codec;
        uint8_t    codec_param;
        uint8_t    stream_seq;
        uint8_t    total_data;
        uint8_t    parity_count;
        uint8_t    chunk_size;
        bool       chunk_size_known;       // chunk_size is inferred per spec §4
        size_t     last_data_real_size;    // <= chunk_size; set when the final DATA arrives
        bool       last_data_seen;
        uint8_t    received_data_count;
        uint32_t   started_ms;
        std::vector<std::vector<uint8_t>> data_shards;    // [total_data][chunk_size]
        std::vector<std::vector<uint8_t>> parity_shards;  // [parity_count][chunk_size]
        std::vector<bool> data_received;                  // [total_data]
        std::vector<bool> parity_received;                // [parity_count]
    };
    AssemblyState states_[MAX_IN_PROGRESS];

    // Simple ring of completed messages.
    ReceivedVoiceMessage complete_[MAX_COMPLETE_QUEUE];
    size_t complete_head = 0;   // pop point
    size_t complete_tail = 0;   // push point
    size_t complete_count = 0;

    AssemblyState *findExisting(NodeNum from, uint32_t message_id);
    AssemblyState *allocateSlot(NodeNum from, uint32_t message_id);
    void           resetSlot(AssemblyState &s);

    // True if `s` now has every data shard (post-FEC if possible) and we can
    // finalize. Caller is responsible for moving the message to complete_ ring.
    bool tryFinalize(AssemblyState &s);

    // Push a finalized AssemblyState's reconstructed audio into complete_ ring.
    void publish(AssemblyState &s);
};

} // namespace voicetastic

#endif // ARCH_ESP32 && !MESHTASTIC_EXCLUDE_VOICETASTIC
