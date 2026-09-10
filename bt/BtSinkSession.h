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
    // --- PAIRING MODE (NEW-46) -----------------------------------------------------------------------------
    // A bonded sink is not discoverable by design (a phone that knows us PAGES).  The consequence the bench
    // recorded twice is that a phone which has FORGOTTEN the sink has no way back except a reflash -- and
    // NEW-43 sharpened it: one failed SSP leaves the controller in legacy-PIN mode until PREPARE runs again.
    // A pairing WINDOW answers both, the way a speaker does: discoverable for a while after power-on and after
    // any attempt ends (loss, clean close, OR a failed pairing -- the NEW-43 path is a failed attempt), and on
    // demand.  Every window ASKS for PREPARE and opens whether or not it gets one: openWindow() calls
    // startPrepare(), which DECLINES while another op is in flight, and returns true either way -- so what
    // re-arms SSP is which op can be running at that moment, not the call.  At begin() that op is the session's
    // own PREPARE (nothing doubled, nothing missed); the drop and commanded windows open from LISTENING, with
    // the attempt over.  P2/P3/P4 COUNT the re-issue at the fake controller rather than assuming it.
    // The window is a DEADLINE beside LISTENING, not a State: MANUAL and LISTENING both compose with it and
    // every `m_state == LISTENING` test in tick() stays as it is.  A window never SURVIVES a link reaching
    // STREAMING (the transition closes it PAIRED, ahead of the callbacks), and enterPairing() refuses rather
    // than queues while a link is up.  It CAN be open through CONNECTING, deliberately -- a page that fails to
    // pair must not have closed the window it is about to need -- and that is not discoverable either way,
    // since the scan line reads `listening`, which a link coming up has already made false.
    enum PairingReason : uint8_t { PAIR_NONE, PAIR_BOOT, PAIR_DROP, PAIR_CMD };
    enum PairingEnd    : uint8_t { PAIR_END_NONE, PAIR_END_TIMEOUT, PAIR_END_PAIRED, PAIR_END_CANCELLED };   // CANCELLED: torn down by the app's disconnect() before it ever saw STREAMING -- not by pairing, not by the clock
    static const uint32_t PAIR_DEFAULT_MS = 120000;
    // AUTOMATIC-window length; 0 switches the automatic windows off and leaves enterPairing() working at
    // PAIR_DEFAULT_MS.  Affects the NEXT window; an open one runs to its deadline.
    void          setPairingWindowMs(uint32_t ms) { m_pairMs = ms; }
    // Idle enough to OPEN a window: LISTENING with no link up, the one condition every trigger tests before it
    // fires.  NOT "a phone can pair with us right now" -- that also needs a window open, or no bond yet.
    bool          canPair() const;
    // Open a commanded window, or extend an open one to now + window; false = refused (see canPair).  The
    // reason is the caller's LABEL and never a policy switch: a window entered HERE is a command whatever it is
    // labelled, so setPairingWindowMs(0) does not refuse it, and PAIR_NONE -- which would leave pairingOpen()
    // and pairingReason() contradicting each other -- normalises to PAIR_CMD (P7).
    bool          enterPairing(uint32_t now, PairingReason r = PAIR_CMD);
    bool          pairingOpen() const { return m_pairOpen; }           // as of the last tick(now): that is where expiry is evaluated, so the poll, the scan line and the heartbeat agree
    uint32_t      pairingRemainingMs(uint32_t now) const { return m_pairOpen && (int32_t)(m_pairUntil - now) > 0 ? m_pairUntil - now : 0; }
    PairingReason pairingReason() const { return m_pairOpen ? m_pairReason : PAIR_NONE; }
    PairingEnd    pairingEnd() const { return m_pairEnd; }             // why the LAST window closed; the sketch prints it on the falling edge
    void onStream(StreamFn fn, void *ctx) { m_streamCb = fn; m_streamCtx = ctx; }
    void onAttempt(AttemptFn fn, void *ctx) { m_attemptCb = fn; m_attemptCtx = ctx; }
    State state() const { return m_state; } const Stats &stats() const { return m_stats; }
    static const char *stateName(State s);
private:
    A2dpSink &m_sink; BondTable *m_bonds = nullptr; State m_state = IDLE; bool m_alwaysDisc = false; Stats m_stats{};
    StreamFn m_streamCb = nullptr; void *m_streamCtx = nullptr; AttemptFn m_attemptCb = nullptr; void *m_attemptCtx = nullptr;
    uint32_t m_pairMs = PAIR_DEFAULT_MS, m_pairUntil = 0; bool m_pairOpen = false;
    PairingReason m_pairReason = PAIR_NONE; PairingEnd m_pairEnd = PAIR_END_NONE;
    // The ONE path every trigger takes.  `commanded` is the app asking (enterPairing) rather than the session
    // deciding (begin, the drop branches) -- it is what setPairingWindowMs(0) switches off, and it is an
    // argument of its own rather than `r == PAIR_CMD` so that a public enum cannot select policy (P7).
    bool openWindow(uint32_t now, PairingReason r, bool commanded);
    // ... and the ONE path every close takes, so the three sites cannot spell it three ways.  Guarded: a close
    // with nothing open leaves the LAST window's end as it was (Q4, P6).
    void closeWindow(PairingEnd e) { if (m_pairOpen) { m_pairOpen = false; m_pairEnd = e; } }
};
