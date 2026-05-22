#pragma once

#include "configuration.h"

#if defined(ARCH_ESP32) && defined(HAS_VOICETASTIC) && !MESHTASTIC_EXCLUDE_VOICETASTIC

#include "VtProtocol.h"
#include "mesh/MeshTypes.h"
#include <stdint.h>
#include <vector>

namespace voicetastic {

// One reassembled voice message, ready for playback or hand-off.
struct ReceivedVoiceMessage {
    NodeNum   from;
    NodeNum   to;                  // MeshPacket destination (broadcast or this node)
    uint8_t   channel;             // MeshPacket channel index
    uint32_t  message_id;
    CodecId   codec;
    uint8_t   codec_param;
    uint8_t   total_data;
    uint8_t   received_data;       // data shards that arrived directly
    uint8_t   recovered_via_fec;   // data shards reconstructed by RS
    uint32_t  completed_ms;
    bool      played = false;      // set by the module after at least one playback
    std::vector<uint8_t> audio;    // reassembled codec frame bytes (no container)
};

// Per-(from, message_id) reassembly state machine. Implements the data-path
// of spec §9: chunk_size inference, FEC recovery, partial-timeout, and the
// quiet-window NACK loop (RX-side emission only — the TX side wires inbound
// NACKs into the module's send queue).
class VtAssembler {
  public:
    VtAssembler() = default;

    // Accept a parsed v2 frame. Returns true iff the call finalized a new
    // message (popComplete() is now non-empty). `to` / `channel` come from
    // the MeshPacket envelope and ride along on the eventual
    // ReceivedVoiceMessage so the UI can route the chat bubble correctly.
    bool acceptFrame(NodeNum from, NodeNum to, uint8_t channel,
                     const VtHeader &h, const uint8_t *body, size_t body_len);

    // Drop stuck assemblies older than the timeout (called from runOnce).
    void tick(uint32_t now_ms);

    // Pop the oldest completed message into `out`. Returns false if none.
    bool popComplete(ReceivedVoiceMessage &out);

    // Diagnostic counts.
    size_t inProgressCount() const;
    size_t completeQueueDepth() const { return complete_count; }

    // Pending NACK to emit on behalf of an in-progress assembly that has been
    // quiet for ≥ NACK_QUIET_MS and still has missing data shards. The caller
    // (VoicetasticModule) builds a v2 NACK frame from these fields, paces it
    // per modem preset, and ships it via service->sendToMesh. Fields mirror
    // the originating message so the sender can route it back to its active
    // TX state.
    struct PendingNack {
        NodeNum   from = 0;       // recipient of the NACK (the original sender)
        NodeNum   to = 0;         // local node — echoed back as the NACK frame's `to`
        uint8_t   channel = 0;
        uint32_t  message_id = 0;
        CodecId   codec = CodecId::CODEC2;
        uint8_t   codec_param = 0;
        uint8_t   stream_seq = 0;
        uint8_t   total_data = 0;
        uint8_t   parity_count = 0;
        bool      give_up = false; // set if NACK_MAX_ROUNDS exhausted
        // Data-only bitmap: bit i ⇒ DATA chunk i is missing. Length = total_data.
        // We deliberately do NOT signal missing PARITY shards. With RS-over-GF(2^8)
        // the assembler can finalize as soon as it holds `total_data` shards in
        // any combination of DATA + PARITY, so the sender only needs to top us up
        // to `total_data` shards. Retransmitting the missing DATA shards always
        // satisfies that condition (and lands us in the FEC-free direct path); a
        // parity-aware bitmap would let the sender substitute fresh parity for a
        // lost data shard, but at no airtime saving vs. resending the data
        // directly. Keeping the bitmap data-only also keeps the wire format
        // bit-identical to voicetastic-desktop's NACK frame.
        std::vector<bool> missing;
    };

    // Returns true and fills `out` if any in-progress assembly is ready to
    // NACK. Mutates the assembly's nack-tracking state (rounds, last_chunk_ms)
    // as a side effect so the same assembly is not re-emitted on the next tick.
    // Caller is expected to send the resulting frame.
    bool pollPendingNack(PendingNack &out, uint32_t now_ms);

  private:
    // In-progress slots are keyed by (from, message_id), so this is the
    // total number of concurrent inbound assemblies across all senders, not
    // a per-sender quota. Eviction is FIFO (allocateSlot evicts the oldest
    // by `started_ms` when the table is full), with the evicted key going
    // into the blacklist so its late drain shards don't restart it.
    static constexpr int  MAX_IN_PROGRESS = 4;
    static constexpr int  MAX_COMPLETE_QUEUE = 4;     // recent received messages
    static constexpr uint32_t TIMEOUT_MS = 30000;     // absolute drop-dead for a stuck assembly

    // NACK loop (spec §9). After NACK_QUIET_MS of silence on an in-progress
    // assembly with missing shards we emit a NACK; we retry up to NACK_MAX_ROUNDS
    // times before giving up and finalizing partial. Numbers are smaller than
    // the desktop's defaults (3000ms / 400 rounds) to fit embedded heuristics:
    // the in-progress timeout above caps total time regardless.
    static constexpr uint32_t NACK_QUIET_MS   = 3000;
    static constexpr uint16_t NACK_MAX_ROUNDS = 8;

    // Completion-memory blacklist (spec §9.1). After we finalize a message —
    // complete OR partial-timeout — we remember (from, message_id) for
    // BLACKLIST_TTL_MS so late-arriving shards from the sender's drain queue
    // can't resurrect a phantom partial reassembly. Spec recommends 600 s;
    // BLACKLIST_MAX is sized below the spec's 100 to bound RAM on the device.
    static constexpr int      BLACKLIST_MAX = 16;
    static constexpr uint32_t BLACKLIST_TTL_MS = 600000;

    struct BlacklistEntry {
        bool     in_use = false;
        NodeNum  from = 0;
        uint32_t message_id = 0;
        uint32_t expires_ms = 0;
    };
    BlacklistEntry blacklist_[BLACKLIST_MAX];
    size_t         blacklist_next = 0;   // FIFO write cursor; replaces oldest when full

    bool isBlacklisted(NodeNum from, uint32_t message_id, uint32_t now_ms);
    void addToBlacklist(NodeNum from, uint32_t message_id, uint32_t now_ms);

    struct AssemblyState {
        bool       in_use = false;
        NodeNum    from;
        NodeNum    to;
        uint8_t    channel;
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
        uint32_t   last_chunk_ms;          // millis() of the last accepted DATA/PARITY frame; drives NACK quiet timer
        uint16_t   nack_rounds;            // NACKs already emitted for this assembly (capped at NACK_MAX_ROUNDS)
        std::vector<std::vector<uint8_t>> data_shards;    // [total_data][chunk_size]
        std::vector<std::vector<uint8_t>> parity_shards;  // [parity_count][chunk_size]
        std::vector<bool> data_received;                  // [total_data]
        std::vector<bool> parity_received;                // [parity_count]

        // Sidecar for the lone final-DATA case (spec §4). If a final-DATA frame
        // arrives before chunk_size has been inferred, its body bytes are
        // stashed here verbatim. As soon as chunk_size becomes known (via a
        // later PARITY or non-final DATA), the sidecar is folded into
        // data_shards[total_data-1] and cleared. Empty otherwise.
        std::vector<uint8_t> last_data_pending;
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
