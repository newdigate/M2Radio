/* WiFiConnPool.h - fixed pool of TCP connection slots for the WiFi facade.
 *
 * Copyright (c) 2026 Nicholas Newdigate
 * SPDX-License-Identifier: MIT
 *
 * THE ownership rule that makes the raw-API discipline structural: lwip's
 * tcp_arg points at a pool slot, NEVER at a WiFiClient.  WiFiClient is a
 * refcounted handle; destructing every handle cannot dangle anything lwip
 * holds.  4 slots against MEMP_NUM_TCP_PCB=5 -- the spare is TIME_WAIT
 * headroom (listeners draw from MEMP_NUM_TCP_PCB_LISTEN).
 */
#pragma once
#include <stdint.h>
#include "lwip/tcp.h"

static const uint8_t WIFI_MAX_CONNS = 4;

// Cap on the pbuf COUNT this pool STAGES for one slot.  Read the bound
// carefully -- it is not "total pbufs in use for this connection".
//
// Why a count at all: the byte bound is real (rcv_wnd closes at TCP_WND =
// 8*MSS = 11680) but it bounds the wrong quantity.  The netif allocates one
// PBUF_POOL pbuf per FRAME regardless of frame size (Iw416Netif.cpp,
// frameSink) and PBUF_POOL_SIZE is 32, so a peer sending 100-byte segments
// would park ~117 pbufs on a single chain -- ~4x the whole pool -- long before
// TCP_WND is approached.  A dry pool makes frameSink drop EVERY inbound frame:
// ARP, DHCP renew, other connections' ACKs.  That is the unbounded case this
// number exists to remove.
//
// What it actually bounds: a non-empty chain never grows past this many pbufs.
// Two things sit OUTSIDE it, both deliberate:
//   - An empty chain accepts whatever lwip hands over in one go, however long.
//     That exemption is what makes permanent refusal impossible (see connRecv),
//     and it costs nothing real: a delivery can be a multi-pbuf chain only
//     because tcp_in.c pbuf_cat'ed queued out-of-order segments (TCP_OOSEQ_MAX_
//     PBUFS is 0 = unlimited here), and those pbufs are ALREADY allocated --
//     refusing them moves ownership to lwip, it does not free anything.
//   - Whatever lwip holds as pcb->refused_data for this pcb, which is what a
//     refusal creates.  So per stalled slot the pool can hold up to this many
//     staged plus one refused delivery, not this many total.
// The honest headline: 4 slots cannot silently consume the pbuf pool through
// staged data, and no single connection can run it dry.
static const uint8_t WIFI_RX_MAX_PBUFS = 6;

struct WiFiConn {
    enum State : uint8_t {
        FREE = 0,
        CONNECTING,      // client connect in flight
        ESTABLISHED,
        PEER_CLOSED,     // peer FIN'd or errored; rx chain still readable.
    };                   // pcb==nullptr => lwip side already gone.
    State    state = FREE;
    struct tcp_pcb *pcb = nullptr;
    struct pbuf *rxHead = nullptr;   // unconsumed RX chain (we own it)
    uint16_t rxOff = 0;              // read offset into rxHead's first pbuf
    uint8_t  refs = 0;               // WiFiClient handles attached
    bool     claimed = false;        // a sketch-visible handle ever existed
    uint16_t serverPort = 0;         // owning WiFiServer port; 0 = client conn
    uint32_t lastActivityMs = 0;
    volatile bool connectDone = false;
    volatile bool connectOk   = false;
};

// ---------------------------------------------------------------------------
// CONTRACT for WiFiClient / WiFiServer -- five things this file's caller must
// know, written here because that author may read only this file.
//
// 1. addRef() immediately after alloc(), before anything that can fail.  See
//    the ordering rule on addRef() below; it is the difference between a
//    failed connect that frees its slot and one that leaks it forever.
// 2. installCallbacks() BEFORE tcp_connect().  It is what stores c->pcb, and
//    until it runs the pool cannot see the pcb at all -- abortAll() skips a
//    slot whose pcb is null, so a link lost mid-connect would strand it.
// 3. The pool does NOT install tcp_connected; the caller passes its own to
//    tcp_connect(), and `arg` there will be the SLOT (installCallbacks set
//    tcp_arg).  connectDone/connectOk are only ever set here to (true,false),
//    by connErr and abortAll -- the success half belongs to that callback.
//    That callback also owns the STATE transition: alloc() reserves the slot
//    as CONNECTING and nothing in the pool advances it, so a connected
//    callback must set state = ESTABLISHED itself.  (The accept path differs
//    -- a server sets ESTABLISHED at accept time, which is why the example
//    below does not show this.)  A slot left in CONNECTING after a successful
//    connect still works for RX/TX, but reads as "connecting" to anything
//    that inspects state, so set it.
// 4. stop() will usually put an RST on the wire, not a FIN.  tcp_recved() is
//    deferred to consume(), so any unread byte leaves rcv_wnd != TCP_WND_MAX,
//    and tcp_close_shutdown() resets rather than closes gracefully (tcp.c).
//    Intended -- it is the flow control working -- but it shows up in captures
//    and in a peer's logs, so do not read it as a bug.
// 5. availableBytes() means "staged right now", NOT "everything the peer
//    sent".  With the cap above, `if (available() >= N)` can stall forever at
//    ~6 segments; consume incrementally (`while (available())`) instead.
// ---------------------------------------------------------------------------
namespace WiFiPool {
    WiFiConn *slot(uint8_t i);           // 0..WIFI_MAX_CONNS-1
    // Reserves the slot it returns (state := CONNECTING) rather than handing
    // back a FREE one and trusting the caller to claim it.  Without that, two
    // consecutive alloc() calls returned the SAME slot -- two WiFiClients on
    // one tcp_arg, two refcount paths, presenting on silicon as cross-talk.
    // The caller overwrites state as it likes (accept -> ESTABLISHED).
    WiFiConn *alloc();                   // reserved slot, or nullptr
    // alloc(); when full, evict the least-recently-active accepted-but-never-
    // claimed slot (refs==0 by definition -- the sketch never saw it).
    // Claimed connections are NEVER evicted.
    WiFiConn *allocEvicting();
    // ORDERING RULE, and it is MANDATORY, not one of two options: addRef()
    // immediately after alloc(), before tcp_new(), before tcp_connect(),
    // before anything that can fail.  Setting `claimed` is an ADDITION to that,
    // never a substitute.  Two independent reasons:
    //   - the stall valve tests both, so either would spare the slot; but
    //   - only a refcount gives the slot a way BACK.  release() is the only
    //     exit, and a slot that was reserved and then abandoned with refs==0
    //     and claimed=true is invisible to every other reaper (abortAll skips
    //     a null pcb, the valves need callbacks installed, the evictor needs
    //     serverPort != 0) -- four failed connects and the pool is dead until
    //     reboot.  release() now also collects such a slot defensively, but
    //     do not rely on that: addRef first.
    //
    // ★ THE ACCEPT PATH IS THE ONE EXCEPTION, and it is not a relaxation --
    // addRef() there would be a BUG.  An accepted-but-unclaimed conn has
    // refs == 0 BY DESIGN: that is precisely the state connPoll's stall valve
    // (!claimed && refs == 0) and the evictor (!claimed && serverPort != 0)
    // exist to reap, because the sketch has never seen it and nothing else
    // will ever free it.  addRef() with no handle to match it would make an
    // accept immune to BOTH valves and leak it permanently -- the inverse of
    // the bug the rule above prevents.  So an accept callback must instead:
    //   - set serverPort != 0 BEFORE anything that can fail (that is what
    //     makes the slot visible to the evictor), and
    //   - leave claimed == false and refs == 0 until a WiFiClient handle
    //     actually adopts it, which is where addRef finally happens.
    // The rule and the exception are the same principle stated twice: a
    // reserved slot must always be reachable by SOME reaper.  For a client
    // connect the only reaper is release(), so it needs a refcount; for an
    // accept the reapers are the valves, and they need the absence of one.
    void addRef(WiFiConn *c);
    // Drop a handle; frees the slot when refs==0 and the conn is dead.  Also
    // the exit for a RESERVED-but-abandoned slot: called on a slot with no
    // handles and no pcb, it returns it to FREE, so every alloc() failure path
    // in a caller can end with one symmetrical release().  A slot that still
    // has a pcb is a live connection and is never dropped this way.
    void release(WiFiConn *c);
    // Clear EVERY callback BEFORE tcp_close; tcp_abort on close failure.
    // Returns what an in-callback caller must return to lwip (ERR_ABRT after
    // the abort path -- returning ERR_OK there leaves tcp_input on a freed
    // pcb; see m2_lwip_test.cpp closeEcho).
    err_t closeConn(WiFiConn *c);
    void abortAll();                     // link lost: no FIN possible
    void installCallbacks(WiFiConn *c, struct tcp_pcb *pcb);
    int  availableBytes(const WiFiConn *c);
    int  peekByte(const WiFiConn *c);
    int  consume(WiFiConn *c, uint8_t *buf, int len);  // + tcp_recved
    // Silicon-visible counters, one per safety valve.  Both exist because a
    // connection that vanishes on its own must leave evidence: evictions() is
    // the accept-path valve (an unclaimed accept dropped to make room),
    // stallAborts() is the idle-path valve (connPoll reaping a stalled,
    // unheld, unclaimed conn at 30-40 s).
    uint32_t evictions();
    uint32_t stallAborts();
}
