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
    WiFiConn *alloc();                   // FREE slot or nullptr
    // alloc(); when full, evict the least-recently-active accepted-but-never-
    // claimed slot (refs==0 by definition -- the sketch never saw it).
    // Claimed connections are NEVER evicted.
    WiFiConn *allocEvicting();
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
