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

// Cap on the pbuf COUNT of one slot's unconsumed RX chain.  The byte bound is
// real (rcv_wnd closes at TCP_WND = 8*MSS = 11680) but it bounds the wrong
// quantity: the netif allocates ONE PBUF_POOL pbuf per FRAME regardless of
// frame size (Iw416Netif.cpp, frameSink), and PBUF_POOL_SIZE is 32.  A peer
// sending 100-byte segments therefore parks ~117 pbufs on one chain before
// TCP_WND is approached -- roughly 4x the entire pool -- and even at full MSS,
// 4 stalled slots x 8 segments = 32 = the whole pool exactly.  A dry pool makes
// frameSink drop EVERY inbound frame: ARP, DHCP renew, the other connections'
// ACKs.  It recovers, so this is degradation rather than deadlock, but one
// unread WiFiClient must not be able to stall the whole stack.  Hence a cap
// that is asserted, not hoped for.
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
    // ORDERING RULE for anything that takes a handle on a slot: addRef() FIRST,
    // or set claimed, BEFORE the slot can be polled.  The stall valve frees an
    // unclaimed idle slot, and it tests refs as well as claimed, so either one
    // is enough -- but a slot that is neither is reapable under a live handle,
    // which is the dangling-handle class this pool exists to remove.
    void addRef(WiFiConn *c);
    void release(WiFiConn *c);           // drop a handle; frees the slot when
                                         // refs==0 and the conn is dead
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
    uint32_t evictions();                // silicon-visible safety-valve counter
}
