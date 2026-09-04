#include "Hci.h"
#include <string.h>

const char *Hci::errorName(Error e) {
    switch (e) {
        case OK:           return "ok";
        case TIMEOUT:      return "no_response";
        case FRAMING:      return "framing";
        case NCMD_STARVED: return "ncmd_starved";
        case QUEUE_FULL:   return "queue_full";
        case STATUS:       return "status";
        case BUSY:         return "busy";
    }
    return "unknown";
}

Hci::Hci(HciIo &io) : m_io(io) {
    // The event and ACL registrations are made ONCE, here, and deliberately
    // NOT in begin().  begin() is documented as "reset state; one credit" and
    // is the natural call for re-initialising a link after a bad fault --
    // nulling the callbacks there would silently unhook every asynchronous
    // event and every ACL packet from that point on, with no fault, no
    // counter, and a link that merely looks quiet.  Registration survives a
    // re-init; everything begin() resets is per-link state.
    m_onEvent = nullptr; m_eventCtx = nullptr;
    m_onAcl = nullptr; m_aclCtx = nullptr;
    begin();
}

void Hci::begin() {
    m_parser.reset();
    m_parser.setCallbacks(packetThunk, faultThunk, this);
    m_qHead = 0; m_qCount = 0;
    m_inflight = false; m_inflightOpcode = 0; m_sentAt = 0;
    m_ncmd = 1;                       // Vol 4 Part E 4.4: one command before any reply
    m_timeoutMs = 1000;
    m_resync = false; m_lastByteAt = 0;
    m_timeouts = m_framing = m_starved = m_queueFull = m_late = m_events = m_reclaimed = 0;
    m_lastError = OK;
}

void Hci::packetThunk(void *ctx, uint8_t type, const uint8_t *pkt, size_t len) {
    ((Hci *)ctx)->onPacket(type, pkt, len);
}
void Hci::faultThunk(void *ctx, uint8_t, uint8_t) { ((Hci *)ctx)->onFault(); }

Hci::Error Hci::submit(uint16_t opcode, const uint8_t *params, uint8_t plen, DoneFn done, void *ctx) {
    if (m_qCount >= QUEUE_DEPTH) { m_queueFull++; m_lastError = QUEUE_FULL; return QUEUE_FULL; }
    Cmd &c = m_q[(m_qHead + m_qCount) % QUEUE_DEPTH];
    c.opcode = opcode; c.plen = plen;
    if (plen) memcpy(c.params, params, plen);
    c.done = done; c.ctx = ctx; c.queuedAt = m_io.nowMs();
    m_qCount++;
    dispatch();
    return OK;
}

// Send the head command if a credit is available and the line is in sync;
// otherwise age it, and fail it by name when it has waited a full timeout.
void Hci::dispatch() {
    if (m_inflight || m_qCount == 0) return;
    uint32_t now = m_io.nowMs();
    Cmd &c = m_q[m_qHead];
    if (m_ncmd > 0 && !m_resync) {
        uint8_t hdr[4] = { 0x01, (uint8_t)(c.opcode & 0xFF), (uint8_t)(c.opcode >> 8), c.plen };
        m_io.write(hdr, 4);
        if (c.plen) m_io.write(c.params, c.plen);
        m_inflight = true; m_inflightOpcode = c.opcode; m_sentAt = now;
        m_ncmd--;
        return;
    }
    if (now - c.queuedAt >= m_timeoutMs) {
        // Name the reason we could not send, and test the resync FIRST.  Both
        // conditions can hold at once -- a resync is also why a credit never
        // came back, since the reply carrying it is among the bytes being
        // discarded -- and reporting that as ncmd_starved sends the reader to
        // the credit accounting when a framing fault is what actually
        // happened.  For a class whose whole claim is that every exit has a
        // name, the wrong name is worse than a bare error code.
        if (m_resync)         { finish(FRAMING, nullptr); }      // the line never went quiet
        else if (m_ncmd == 0) { m_starved++; finish(NCMD_STARVED, nullptr); }
        else                  { finish(FRAMING, nullptr); }
    }
}

void Hci::service() {
    uint32_t now = m_io.nowMs();
    while (m_io.available() > 0) {
        int b = m_io.read();
        if (b < 0) break;
        m_lastByteAt = now;
        if (m_resync) continue;                  // discard until the line has been idle
        m_parser.feed((uint8_t)b);
    }
    // Re-read the clock AFTER the drain.  A packet delivered by the drain can
    // run a callback that submit()s a command, and dispatch() stamps m_sentAt
    // with a fresh clock -- later than the `now` captured above once the drain
    // has spent a few milliseconds (printing inside a callback is enough).  The
    // unsigned (now - m_sentAt) below then underflows and the command times out
    // in the SAME pass it was sent: measured on silicon as SSP replies "timing
    // out" 3-4 ms after submit and their prompt acks counted as late.  The
    // stale `now` is still the right stamp for the bytes above (it is never
    // later than they are), so only the checks move to the fresh clock.
    now = m_io.nowMs();
    if (m_resync && (now - m_lastByteAt) >= IDLE_RESYNC_MS) { m_resync = false; m_parser.reset(); }
    if (m_inflight && (now - m_sentAt) >= m_timeoutMs) {
        m_timeouts++;
        // Give back the credit dispatch() spent on the command we are about to
        // abandon.  This is the SAME absolute-assignment deadlock onFault()
        // describes below, reached down a different road: m_ncmd is assigned
        // ABSOLUTELY from each reply, so a reply that never arrives leaves the
        // count stuck low with nothing able to raise it -- no credit means no
        // command, and no command means no reply.
        // Measured, before this line existed, by m2_hci_probe -- which retries
        // HCI Reset ten times because the card's settle time after firmware
        // download is unknown -- against a QEMU run with no controller at all:
        //   hci_reset=fail reason=ncmd_starved attempts=10 timeouts=1 framing=0 starved=9 qfull=0 late=0
        // ONE command reached the wire.  Attempt 1 spent the single startup
        // credit and timed out without returning it; attempts 2-10 never
        // dispatched and aged out as NCMD_STARVED.  The retry loop was
        // decoration, and a card that answered on attempt 3 would have been
        // reported dead -- the exact wrong answer from a probe whose only job
        // is to find out whether Bluetooth answers.
        // This restores only what dispatch() spent (it cannot fire unless we
        // sent), so it cannot invent a credit that was never ours.  Over by one
        // is bounded at one and the next reply reassigns the true count; a
        // deadlock does not self-correct.  Guarded against wrap for the reason
        // onFault() gives.
        if (m_ncmd < 0xFF) m_ncmd++;
        finish(TIMEOUT, nullptr);
    }
    dispatch();
}

// Complete the HEAD command (in flight or still waiting) and pop it.
void Hci::finish(Error e, const Reply *r) {
    DoneFn done = m_q[m_qHead].done; void *ctx = m_q[m_qHead].ctx;
    m_qHead = (uint8_t)((m_qHead + 1) % QUEUE_DEPTH); m_qCount--;
    m_inflight = false;
    if (e != OK) m_lastError = e;
    if (done) done(ctx, e, r);
}

void Hci::onPacket(uint8_t type, const uint8_t *pkt, size_t len) {
    if (type == H4Parser::ACL) {
        if (len < 4) return;
        uint16_t handle = (uint16_t)((pkt[0] | (pkt[1] << 8)) & 0x0FFF);
        uint16_t dlen   = (uint16_t)(pkt[2] | (pkt[3] << 8));
        if (m_onAcl) m_onAcl(m_aclCtx, handle, pkt + 4, dlen);
        return;
    }
    if (len < 2) return;
    uint8_t code = pkt[0], plen = pkt[1];
    const uint8_t *p = pkt + 2;
    if (code == 0x0E && plen >= 3) {                             // Command Complete
        m_ncmd = p[0];
        uint16_t opcode = (uint16_t)(p[1] | (p[2] << 8));
        if (m_inflight && opcode == m_inflightOpcode) {
            m_scratch.statusEvent = false;
            m_scratch.status = plen >= 4 ? p[3] : 0;
            m_scratch.len    = plen >= 4 ? (uint8_t)(plen - 4) : 0;
            if (m_scratch.len) memcpy(m_scratch.params, p + 4, m_scratch.len);
            finish(m_scratch.status == 0 ? OK : STATUS, &m_scratch);
        } else if (opcode != 0x0000) {                           // 0x0000 = NOP: credit only
            m_late++;
        }
        return;
    }
    if (code == 0x0F && plen >= 4) {                             // Command Status
        uint8_t status = p[0]; m_ncmd = p[1];
        uint16_t opcode = (uint16_t)(p[2] | (p[3] << 8));
        if (m_inflight && opcode == m_inflightOpcode) {
            m_scratch.statusEvent = true; m_scratch.status = status; m_scratch.len = 0;
            finish(status == 0 ? OK : STATUS, &m_scratch);
        } else if (opcode != 0x0000) {
            m_late++;
        }
        return;
    }
    m_events++;
    if (m_onEvent) m_onEvent(m_eventCtx, code, p, plen);
}

void Hci::onFault() {
    m_framing++;
    // Do NOT stamp m_lastByteAt here.  service()'s drain loop has already
    // stamped it, with the same `now` it captured at the top of the pass, for
    // this very byte -- the one whose parse faulted.  Re-reading the clock can
    // land a millisecond later, making m_lastByteAt LATER than that `now`, and
    // the unsigned (now - m_lastByteAt) in service() then underflows to a huge
    // value that satisfies the idle test and clears the resync in the SAME
    // pass.  The next command would be dispatched into a still-desynced
    // stream, defeating the only recovery an H4 link has.  Test case 14 covers
    // this with its own TickingIo, whose read() advances the clock per byte --
    // the shared FakeIo cannot express it, because that clock moves only
    // inside idle(), never during a drain.
    m_resync = true;
    if (m_inflight) {
        // Give back the credit dispatch() spent on the command we are about to
        // kill.  m_ncmd is assigned ABSOLUTELY from each reply, so if the reply
        // that carried the credit is one of the bytes the resync discards, the
        // count is stuck low with nothing able to raise it: no credit means no
        // command, and no command means no reply.  One garbage burst would wedge
        // the link for good -- and on this board it is a garbage burst that is
        // expected, LPUART2 having no usable flow control.
        // This restores only what we spent (it cannot fire unless we sent), so it
        // cannot invent a credit that was never ours.  It CAN leave us one too
        // many if the lost reply was going to grant zero; the next reply
        // reassigns the true count, and over by one self-corrects where a
        // deadlock does not.  The nothing-in-flight case needs no help: a
        // controller freeing a buffer sends a NOP (opcode 0x0000) Command
        // Complete carrying the credit, which onPacket() already accepts.
        // The wrap guard is not paranoia: the grant is ABSOLUTE and assigned
        // before the opcode match, so a reply arriving while we are in flight
        // can legitimately set m_ncmd to 255, and m_ncmd is uint8_t.  An
        // unguarded ++ would roll 255 -> 0 and manufacture exactly the
        // starvation deadlock this restore exists to prevent -- and would make
        // the "one too many" above wrong in that one corner.
        if (m_ncmd < 0xFF) m_ncmd++;
        finish(FRAMING, nullptr);
    }
}

namespace {
struct RunCtx { Hci::Reply *reply; bool done; Hci::Error err; };
void runDone(void *ctx, Hci::Error e, const Hci::Reply *r) {
    RunCtx *c = (RunCtx *)ctx;
    if (r && c->reply) *c->reply = *r;
    c->err = e; c->done = true;
}
}

Hci::Error Hci::run(uint16_t opcode, const uint8_t *params, uint8_t plen, Reply *reply,
                    uint32_t timeoutMs, void (*idle)()) {
    if (busy()) { m_lastError = BUSY; return BUSY; }
    if (reply) { reply->status = 0xFF; reply->statusEvent = false; reply->len = 0; }
    RunCtx c = { reply, false, OK };
    uint32_t saved = m_timeoutMs; m_timeoutMs = timeoutMs;
    Error e = submit(opcode, params, plen, runDone, &c);
    if (e != OK) { m_timeoutMs = saved; return e; }
    while (!c.done) { service(); if (!c.done && idle) idle(); }
    m_timeoutMs = saved;
    return c.err;
}
