/* WiFiClient.h - Arduino Client over a WiFiConnPool slot.
 *
 * Copyright (c) 2026 Nicholas Newdigate
 * SPDX-License-Identifier: MIT
 *
 * A refcounted HANDLE, never an owner: lwip's tcp_arg points at the pool SLOT
 * (WiFiConnPool.h), so destroying every handle cannot dangle a callback and the
 * use-after-free class is designed out structurally rather than guarded
 * against.  Copies share the connection, which is what gives Arduino's
 * shared-socket semantics (`WiFiClient c = server.available();`) for free.
 *
 * available()/read()/peek()/connected() service the link when they find NOTHING
 * STAGED, so the classic `while (c.connected()) if (c.available()) ...` sketch
 * loop still polls the SDIO card by construction -- a sketch that never calls
 * WiFi.loop() and never delay()s still gets its bytes.  They deliberately do
 * NOT service when bytes are already staged: a service pass ends in the
 * driver's trailing delay(1) (WiFi.h's ~1 kHz note), so servicing on every call
 * would put ~1 ms between consecutive read()s and cap a drain at ~1 kB/s --
 * with everything on Stream (readBytes/readString/parseInt) funnelling through
 * one-byte timedRead(), that is the whole class, not one idiom.
 *
 * Caller-visible consequences of the pool's contract, restated here because
 * they surprise people (WiFiConnPool.h has the full text):
 *   - available() is "staged RIGHT NOW", not "everything the peer sent".  The
 *     pool caps a staged chain at WIFI_RX_MAX_PBUFS, so `if (c.available() >=
 *     N)` can stall forever for a large N.  Consume incrementally:
 *     `while (c.available()) ...`.
 *   - stop() puts an RST on the wire whenever ANY received byte is left
 *     unread -- tcp_recved() is deferred to read(), so an unread byte leaves
 *     the receive window short and tcp.c resets rather than closing gracefully.
 *     Drain first and it FINs normally.  Either way it is the flow control
 *     working, but it is visible in captures and in a peer's logs.
 *
 * TIMING a caller must budget for: connect() calls stop() first, so on a live
 * client its worst case is the 5 s send-buffer drain PLUS the 10 s connect
 * timeout = 15 s, not the 10 s the .cpp's constant suggests on its own.
 */
#pragma once
#include "Client.h"

struct WiFiConn;

class WiFiClient : public Client {
public:
    // Why connect() failing needs more than `0`: the six failure sites below
    // are indistinguishable in the return value, and a bench reading a
    // transcript cannot tell "the AP is gone" from "the pool is full" from
    // "the peer refused".  Same reasoning as the pool's evictions() and
    // stallAborts() counters -- something that fails on its own must leave
    // evidence.  Precedent for the shape: EthernetClient::status(),
    // WiFiNINA's WiFiClient::status().
    enum ConnectError : uint8_t {
        CONNECT_OK = 0,
        NO_LINK,        // lwip not up, or the radio link is down
        NO_SLOT,        // all WIFI_MAX_CONNS pool slots in use
        NO_PCB,         // lwip is out of tcp_pcbs (MEMP_NUM_TCP_PCB)
        NO_ROUTE,       // tcp_connect() refused: no route / no local port
        TIMED_OUT,      // 10 s with no answer of any kind
        REFUSED,        // RST or unreachable -- connErr fired
        DNS_FAILED,     // connect(host, port) could not resolve
    };

    WiFiClient() : m_conn(nullptr), m_err(CONNECT_OK) {}
    // Pool handoff: adopts an EXISTING slot (WiFiServer's accept path).  Takes
    // its own reference and marks the slot claimed -- a claimed slot is exempt
    // from the pool's eviction scan, which is the point: the sketch has now
    // seen it.
    explicit WiFiClient(WiFiConn *conn);
    WiFiClient(const WiFiClient &other);
    WiFiClient &operator=(const WiFiClient &other);
    virtual ~WiFiClient();

    int connect(IPAddress ip, uint16_t port) override;
    int connect(const char *host, uint16_t port) override;
    size_t write(uint8_t b) override { return write(&b, 1); }
    size_t write(const uint8_t *buf, size_t size) override;
    int available() override;
    int availableForWrite() override;          // room in lwip's send buffer
    int read() override;
    int read(uint8_t *buf, size_t size) override;
    int peek() override;
    void flush() override;                     // drain lwip's send buffer
    void stop() override;
    uint8_t connected() override;
    operator bool() override { return m_conn != nullptr; }
    // Why the LAST connect() returned 0.  Sticky until the next connect().
    uint8_t lastError() const { return (uint8_t)m_err; }
    using Print::write;

private:
    void detach();
    WiFiConn *m_conn;
    ConnectError m_err;
};
