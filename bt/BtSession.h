// BtSession -- the A2DP link-lifecycle policy (NEW-34 piece 2): boot walk + inquiry, lost-peer retry
// forever, page-scan-when-idle, retry-cancel-on-incoming, MANUAL mode, stats, callbacks.  Ticks an
// A2dpSource beneath it.  Arduino-free, no heap; the app persists bonds on the attempt-end callback.  MIT.
#pragma once
#include <stdint.h>
#include "A2dpSource.h"
#include "BondTable.h"
class BtSession {
public:
    enum State : uint8_t { IDLE, CONNECTING, WAITING, STREAMING, DISCONNECTING, MANUAL };
    enum By : uint8_t { BY_NONE, BY_PAGED, BY_INQUIRY, BY_INCOMING };
    struct Stats { uint32_t links, lost, attempts, accepts, rejects; uint8_t lastReason; By by; uint32_t reconnectMs, lostAt; };
    typedef void (*StreamFn)(void *ctx, bool streaming, uint8_t reason, By by);
    typedef void (*AttemptFn)(void *ctx, A2dpSource::Result r, const char *pairedBy);
    explicit BtSession(A2dpSource &src) : m_src(src) {}
    void begin(BondTable *bonds, const char *targetName, uint8_t aclNum, uint32_t now);
    void tick(uint32_t now);
    void disconnect();          // -> DISCONNECTING -> MANUAL
    void resume();              // MANUAL -> boot policy
    void retryNow();            // fire the retry timer now (NEW-35 hook)
    void onStream(StreamFn fn, void *ctx)  { m_streamCb = fn; m_streamCtx = ctx; }
    void onAttempt(AttemptFn fn, void *ctx){ m_attemptCb = fn; m_attemptCtx = ctx; }
    void setRetryMs(uint32_t ms) { m_retryMs = ms; }
    State state() const { return m_state; }
    const Stats &stats() const { return m_stats; }
    bool wantPageScan() const { return m_wantScan; }
private:
    void startBootWalk(); void startReconnect(uint32_t now); void beginAttempt(const A2dpSource::Target &t);
    bool advanceBootWalk(uint32_t now);   // issue the next boot candidate (or inquiry); false when the walk is exhausted
    void handleAttemptEnd(uint32_t now);  // an attempt reached OK/failure: callbacks, then STREAMING or advance/WAITING
    A2dpSource &m_src; BondTable *m_bonds = nullptr; const char *m_target = nullptr; uint8_t m_aclNum = 0;
    State m_state = IDLE; uint32_t m_retryMs = 10000, m_retryAt = 0; bool m_wantScan = false;
    // boot walk cursor
    bool m_boot = true; uint8_t m_walkIdx = 0; bool m_walkFirst = true; bool m_triedInquiry = false;
    // reconnect target (the lost address)
    uint8_t m_lostBd[6] = {0}; bool m_haveLost = false;
    Stats m_stats{}; A2dpSource::Result m_lastResult = A2dpSource::OK;
    StreamFn m_streamCb = nullptr; void *m_streamCtx = nullptr;
    AttemptFn m_attemptCb = nullptr; void *m_attemptCtx = nullptr;
};
