// Hci -- the host side of an HCI link over H4 (Core 5.2 Vol 4 Part E).
//
// A small command queue that honours Num_HCI_Command_Packets, matches
// Command Complete / Command Status to the command in flight by opcode, times
// commands out, and hands everything else (asynchronous events, ACL data) to
// callbacks for the layers above.  Never blocks: service() is one bounded
// pass, and run() is a convenience for probes that loops service() itself.
//
// Every way a command can fail has a NAME and a COUNTER (the WiFiClass
// lastError() idiom), because H4 has no flow control on this board and no
// sync marker: a lost byte desyncs the stream for good.  The parser's fault
// starts a RESYNC -- bytes are discarded until the line has been idle for
// IDLE_RESYNC_MS -- and the command in flight fails as FRAMING, not TIMEOUT.
//
// Fixed pools, no heap.  MIT, clean-room from the specification.
#pragma once
#include <stdint.h>
#include <stddef.h>
#include "H4Parser.h"
#include "HciIo.h"

class Hci {
public:
    enum Error : uint8_t {
        OK = 0,
        TIMEOUT,        // no Command Complete / Command Status within the deadline
        FRAMING,        // a framing fault while in flight, or the line never went quiet
        NCMD_STARVED,   // the controller never granted a command credit
        QUEUE_FULL,     // submit() with QUEUE_DEPTH commands already queued
        STATUS,         // answered, with a non-zero status (see Reply::status)
        BUSY,           // run() while another command is queued or in flight
    };
    static const char *errorName(Error e);

    struct Reply {
        uint8_t status;        // Command Complete: return parameter 0.  Command Status: its status byte.
        bool    statusEvent;   // answered by Command Status (0x0F) rather than Command Complete (0x0E)
        uint8_t len;           // bytes in params (return parameters AFTER the status byte)
        uint8_t params[254];
    };
    // reply is null on failure.  RE-ENTRANCY CONTRACT: `done` fires from
    // inside service(), which means from inside the parser's emit path, with a
    // packet half-processed on the stack.  Calling submit() from it is safe
    // and intended (verified nested four deep -- it only touches the queue and
    // may dispatch).  Calling service() or run() from it is NOT: re-entering
    // the parser corrupts the packet in progress and loses it silently, with
    // no fault raised and no counter moved.  Queue the work and let the outer
    // service() return.
    typedef void (*DoneFn)(void *ctx, Error e, const Reply *reply);
    typedef void (*EventFn)(void *ctx, uint8_t code, const uint8_t *params, uint8_t len);
    typedef void (*AclFn)(void *ctx, uint16_t handle, const uint8_t *data, uint16_t len);

    static const uint8_t  QUEUE_DEPTH    = 4;
    static const uint32_t IDLE_RESYNC_MS = 50;

    explicit Hci(HciIo &io);
    void begin();                        // reset state; one credit (Vol 4 Part E 4.4)
    void service();                      // one bounded pass; never blocks

    // Queue a command.  `done` fires from service() when it completes or fails.
    // A queued command not dispatched within the timeout fails: NCMD_STARVED
    // when the credit count is 0, FRAMING when the line never went quiet.
    Error submit(uint16_t opcode, const uint8_t *params, uint8_t plen, DoneFn done, void *ctx);
    // Probe helper: submit, then loop service() (calling `idle` between passes,
    // e.g. a delay(1)) until the command completes or fails.  Refuses (BUSY) if
    // anything is queued or in flight.
    Error run(uint16_t opcode, const uint8_t *params, uint8_t plen, Reply *reply,
              uint32_t timeoutMs, void (*idle)() = nullptr);

    void onEvent(EventFn fn, void *ctx) { m_onEvent = fn; m_eventCtx = ctx; }
    void onAcl(AclFn fn, void *ctx)     { m_onAcl = fn; m_aclCtx = ctx; }
    void setAclMax(uint16_t max)        { m_parser.setAclMax(max); }   // from Read_Buffer_Size
    void setCommandTimeout(uint32_t ms) { m_timeoutMs = ms; }          // for submit(); run() takes its own

    bool     busy()      const { return m_inflight || m_qCount != 0; }
    uint8_t  ncmd()      const { return m_ncmd; }
    uint32_t timeouts()  const { return m_timeouts; }
    uint32_t framing()   const { return m_framing; }      // parser faults seen
    uint32_t starved()   const { return m_starved; }
    uint32_t queueFull() const { return m_queueFull; }
    // Replies to commands already given up on.  This is the one failure the
    // file header's "every failure has a name and a counter" promise does not
    // cover, and it is a real limitation rather than an oversight: matching is
    // by OPCODE ONLY, because HCI carries no transaction id.  So after a
    // TIMEOUT, a stale reply to the abandoned command is byte-for-byte
    // indistinguishable from the reply to its retry, and if the retry is in
    // flight the stale one is attributed to it -- counted as the answer, not
    // as late.  late() therefore counts only the stale replies that arrive
    // with nothing of the same opcode outstanding.
    uint32_t late()      const { return m_late; }
    uint32_t events()    const { return m_events; }       // asynchronous events delivered
    Error    lastError() const { return m_lastError; }
    // Grant ONE command credit locally, only when the count is 0.  For a controller
    // that answered a long-running command's Command Status with
    // Num_HCI_Command_Packets=0 and then never sent the NOP Command Complete that
    // returns it: the host then cannot even send the CANCEL that would clear the
    // controller's state, and every later command ages out as NCMD_STARVED --
    // measured on the bench (2026-09-03) as a wedged link after a silent
    // Create_Connection.  Bounded at one, counted in reclaimed(), and the next
    // reply reassigns the true count.  For callers that have waited far longer
    // than the outstanding command could legitimately take (BtLink's page wait).
    void     reclaimCredit() { if (m_ncmd == 0) { m_ncmd = 1; m_reclaimed++; } }
    uint32_t reclaimed() const { return m_reclaimed; }

private:
    struct Cmd { uint16_t opcode; uint8_t plen; uint8_t params[255]; DoneFn done; void *ctx; uint32_t queuedAt; };
    static void packetThunk(void *ctx, uint8_t type, const uint8_t *pkt, size_t len);
    static void faultThunk(void *ctx, uint8_t fault, uint8_t byte);
    void onPacket(uint8_t type, const uint8_t *pkt, size_t len);
    void onFault();
    void dispatch();
    void finish(Error e, const Reply *r);

    HciIo   &m_io;
    H4Parser m_parser;
    Cmd      m_q[QUEUE_DEPTH];
    uint8_t  m_qHead, m_qCount;
    bool     m_inflight; uint16_t m_inflightOpcode; uint32_t m_sentAt;
    uint8_t  m_ncmd;
    uint32_t m_timeoutMs;
    bool     m_resync; uint32_t m_lastByteAt;
    Reply    m_scratch;
    uint32_t m_timeouts, m_framing, m_starved, m_queueFull, m_late, m_events, m_reclaimed;
    Error    m_lastError;
    EventFn  m_onEvent; void *m_eventCtx;
    AclFn    m_onAcl;   void *m_aclCtx;
};
