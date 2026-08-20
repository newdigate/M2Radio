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
 * available()/read()/peek()/connected() run one bounded service pass first, so
 * the classic `while (c.connected()) if (c.available()) ...` sketch loop
 * services the link by construction -- a sketch that never calls WiFi.loop()
 * and never delay()s still gets its bytes.
 *
 * Two caller-visible consequences of the pool's contract, restated here because
 * they surprise people (WiFiConnPool.h has the full text):
 *   - available() is "staged RIGHT NOW", not "everything the peer sent".  The
 *     pool caps a staged chain at WIFI_RX_MAX_PBUFS, so `if (c.available() >=
 *     N)` can stall forever for a large N.  Consume incrementally:
 *     `while (c.available()) ...`.
 *   - stop() usually puts an RST on the wire rather than a FIN, because
 *     tcp_recved() is deferred to read() and any unread byte leaves the receive
 *     window short (tcp.c closes such a pcb by resetting).  That is the flow
 *     control working; it is visible in captures and in a peer's logs.
 */
#pragma once
#include "Client.h"

struct WiFiConn;

class WiFiClient : public Client {
public:
    WiFiClient() : m_conn(nullptr) {}
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
    int read() override;
    int read(uint8_t *buf, size_t size) override;
    int peek() override;
    void flush() override;                     // drain lwip's send buffer
    void stop() override;
    uint8_t connected() override;
    operator bool() override { return m_conn != nullptr; }
    using Print::write;

private:
    void detach();
    WiFiConn *m_conn;
};
