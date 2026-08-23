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

Hci::Hci(HciIo &io) : m_io(io) { begin(); }

void Hci::begin() {
    m_parser.reset();
    m_parser.setCallbacks(packetThunk, faultThunk, this);
    m_qHead = 0; m_qCount = 0;
    m_inflight = false; m_inflightOpcode = 0; m_sentAt = 0;
    m_ncmd = 1;                       // Vol 4 Part E 4.4: one command before any reply
    m_timeoutMs = 1000;
    m_resync = false; m_lastByteAt = 0;
    m_timeouts = m_framing = m_starved = m_queueFull = m_late = m_events = 0;
    m_lastError = OK;
    m_onEvent = nullptr; m_eventCtx = nullptr;
    m_onAcl = nullptr; m_aclCtx = nullptr;
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
        if (m_ncmd == 0) { m_starved++; finish(NCMD_STARVED, nullptr); }
        else             { finish(FRAMING, nullptr); }          // the line never went quiet
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
    if (m_resync && (now - m_lastByteAt) >= IDLE_RESYNC_MS) { m_resync = false; m_parser.reset(); }
    if (m_inflight && (now - m_sentAt) >= m_timeoutMs) { m_timeouts++; finish(TIMEOUT, nullptr); }
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
    m_resync = true; m_lastByteAt = m_io.nowMs();
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
        m_ncmd++;
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
