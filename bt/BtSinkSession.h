// BtSinkSession -- the SINK's link-lifecycle policy (NEW-41): discoverable + connectable while idle (page scan always,
// inquiry scan until a bond exists or always if asked), take over each accepted incoming link as an A2dpSink attempt,
// scanning off while a link is up, back to LISTENING on loss or a clean stream close.  Reconnect is the SOURCE's job
// (phones page a known sink themselves), so there is no boot walk, no inquiry and no retry timer.  Kept as its OWN
// class rather than a flag on BtSession: two policies, not one with modes (the piece-2 lesson).  Arduino-free, no heap.  MIT.
#pragma once
#include <stdint.h>
#include "A2dpSink.h"
#include "BondTable.h"
class BtSinkSession {
public:
    enum State : uint8_t { IDLE, LISTENING, CONNECTING, STREAMING, DISCONNECTING, MANUAL };
    // links   -- streams that reached STREAMING.  accepts == links BY CONSTRUCTION for a sink (both are
    //            incremented on the one transition, and a sink never pages), so the pair is not two
    //            independent readings the way BtSession's is; it is kept for symmetry with that struct.
    // attempts -- inbound links actually taken over (a start() that refused is not one).
    // rejects  -- attempts that ended without streaming.  closed -- streams the SOURCE closed cleanly.
    // lost     -- streams a link drop ended;  lastReason is that drop's HCI reason.
    // lostAt   -- millis() of the LAST loss, set ONLY on the loss edge.  0 means "no loss yet" AND "a loss
    //            at t=0" -- the two are indistinguishable, so read it only after seeing lost > 0.
    struct Stats { uint32_t links, lost, closed, attempts, accepts, rejects; uint8_t lastReason; uint32_t lostAt; };
    typedef void (*StreamFn)(void *ctx, bool streaming, uint8_t reason);
    typedef void (*AttemptFn)(void *ctx, A2dpSink::Result r, const char *pairedBy);
    explicit BtSinkSession(A2dpSink &sink) : m_sink(sink) {}
    void begin(BondTable *bonds, uint8_t aclNum, uint32_t now);
    void tick(uint32_t now);
    void disconnect();  void resume();
    void setAlwaysDiscoverable(bool on) { m_alwaysDisc = on; }
    void onStream(StreamFn fn, void *ctx) { m_streamCb = fn; m_streamCtx = ctx; }
    void onAttempt(AttemptFn fn, void *ctx) { m_attemptCb = fn; m_attemptCtx = ctx; }
    State state() const { return m_state; } const Stats &stats() const { return m_stats; }
    static const char *stateName(State s);
private:
    A2dpSink &m_sink; BondTable *m_bonds = nullptr; State m_state = IDLE; bool m_alwaysDisc = false; Stats m_stats{};
    StreamFn m_streamCb = nullptr; void *m_streamCtx = nullptr; AttemptFn m_attemptCb = nullptr; void *m_attemptCtx = nullptr;
};
