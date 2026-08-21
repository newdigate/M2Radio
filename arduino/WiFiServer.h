/* WiFiServer.h - Arduino Server over the WiFiConnPool.
 *
 * Copyright (c) 2026 Nicholas Newdigate
 * SPDX-License-Identifier: MIT
 *
 * Accept backpressure + the stall safety valve: with the pool full, an
 * incoming SYN evicts the least-recently-active accepted-but-NEVER-CLAIMED
 * conn (the sketch never saw it, so nothing dangles); if every slot is
 * claimed, the accept is refused (ERR_MEM).  Claimed conns are never touched
 * -- but note the flip side: a sketch holding 4 claimed dead conns starves
 * new accepts until it stop()s them.
 *
 * ★ THE ONE THING TO KNOW BEFORE EDITING acceptCb: an accepted conn is left
 * with refs == 0 and claimed == false ON PURPOSE.  WiFiConnPool.h's ordering
 * rule ("addRef() immediately after alloc()") does NOT apply here and calling
 * addRef() would be a BUG -- refs == 0 && !claimed is precisely the state the
 * pool's two reapers look for (connPoll's stall valve tests !claimed && refs
 * == 0, the evictor tests !claimed && serverPort != 0), so a reference with no
 * handle behind it would make the accept immune to BOTH and leak it forever.
 * What acceptCb owes the pool instead is `serverPort != 0`, set before
 * anything else: that is the flag the evictor scans for.  The rule and this
 * exception are one principle -- a reserved slot must always be reachable by
 * SOME reaper.  Read the ★ block in WiFiConnPool.h; it is authoritative.
 *
 * WHICH CONNECTIONS EACH GETTER SURFACES, because the two are easy to swap:
 *   - accept()    -> any conn this server accepted that the sketch has NEVER
 *                    SEEN, with or without data.  Claiming it (which the
 *                    returned handle does) is what takes it off the evictor's
 *                    list, so a sketch that wants to hold connections open
 *                    across quiet periods must use this one.
 *   - available() -> any conn this server accepted that has BYTES STAGED RIGHT
 *                    NOW, claimed or not.  A connection that connects and then
 *                    says nothing is invisible to available() and will be
 *                    evicted (pool full) or stall-aborted (30-40 s idle) --
 *                    correct Arduino behaviour, and the reason `available()`-
 *                    only sketches cannot hold an idle session.
 * Both CLAIM what they return, and claiming is permanent: a conn handed out by
 * available() will never appear from accept() again.
 *
 * WHY THESE TWO SERVICE THE LINK UNCONDITIONALLY, unlike WiFiClient's readers
 * (which deliberately skip a service pass when bytes are already staged): a
 * service pass is what runs lwip's input path, and lwip's input path is what
 * calls acceptCb -- so for a server, skipping the pass does not just delay
 * data, it delays the CONNECTION.  The cost is the one WiFi.h documents (~1 ms
 * per pass, the driver's trailing delay(1)), paid once per loop() iteration in
 * every idiomatic sketch; WiFiClient's short-circuit exists because Stream
 * funnels readBytes/readString/parseInt through one-byte timedRead(), and a
 * server getter has no such funnel.  It also means a sketch that turned the
 * auto-service pump off and polls only server.available() still keeps the
 * radio serviced.
 *
 * write() BROADCASTS, and the broadcast is all-or-nothing PER CONNECTION and
 * unbuffered: a peer whose send buffer cannot take the whole call loses that
 * copy rather than receiving half a message or stalling the other three.  The
 * return value is therefore "bytes offered", not "bytes every peer got" --
 * getWriteError() is the only signal that somebody missed one.  See the .cpp.
 */
#pragma once
#include <stdint.h>
#include "Server.h"
#include "WiFiClient.h"
#include "lwip/err.h"

struct tcp_pcb;

class WiFiServer : public Server {
public:
    explicit WiFiServer(uint16_t port) : m_port(port) {}
    // Non-copyable: two WiFiServers sharing one listen pcb would double-close
    // it through ~WiFiServer, and there is no refcount here (unlike
    // WiFiClient, which is a handle by design).  A server is a fixture.
    WiFiServer(const WiFiServer &) = delete;
    WiFiServer &operator=(const WiFiServer &) = delete;
    ~WiFiServer() { end(); }

    // Safe to call with no link/lwip: records nothing, stays falsy, no wedge.
    // Idempotent, and RETRYABLE -- a begin() that could not listen leaves
    // m_listen null, so calling it again once WiFi is up succeeds.
    void begin() override;
    WiFiClient available();              // a conn with data pending (claims it)
    WiFiClient accept();                 // any unclaimed accepted conn
    size_t write(uint8_t b) override { return write(&b, 1); }
    size_t write(const uint8_t *buf, size_t size) override;   // broadcast
    // Stops accepting.  Does NOT touch already-accepted connections: handles
    // the sketch holds keep working, and unclaimed ones are left to the pool's
    // stall valve.  Closing the listener cannot close them -- lwip has already
    // handed those pcbs over.
    void end();
    operator bool() { return m_listen != nullptr; }
    uint16_t port() const { return m_port; }
    using Print::write;

private:
    static err_t acceptCb(void *arg, struct tcp_pcb *newpcb, err_t err);
    uint16_t m_port;
    struct tcp_pcb *m_listen = nullptr;
};
