// SdpServer -- answers a peer's SDP requests on a channel the PEER opened at
// us (PSM 0x0001, L2cap::Channel::peerInitiated), using Sdp::serve() and our
// AudioSource record.  RX entry point only RECORDS; the reply is queued from
// service() -- writing to the transport from the RX pump bus-faults (B6,
// 2026-08-28).  One request slot: SDP clients are request/response
// sequential per channel, and a second request landing before service() ran
// replaces the first (counted in dropped()).  MIT, no heap.
#pragma once
#include <stdint.h>
#include "L2cap.h"
class SdpServer {
public:
    // From the L2cap data callback.  Returns true when the payload was an SDP request on a
    // peer-initiated SDP channel (recorded here, answered from service()); false for anything
    // else -- in particular our own SDP CLIENT channel's responses, which the caller parses.
    bool onData(const L2cap::Channel &ch, const uint8_t *p, uint16_t len);
    void service(L2cap &l2);                  // main context: build + queue the pending reply (retried while the TXQ is full)
    bool     pending()  const { return m_pending; }
    uint32_t answered() const { return m_answered; }
    uint32_t dropped()  const { return m_dropped; }
    static const uint16_t MAX_REQ = 64;       // the Shokz's largest request is 21 bytes, the OneOdio's 20
private:
    volatile bool m_pending = false; uint16_t m_cid = 0, m_mtu = 0, m_len = 0; uint8_t m_req[MAX_REQ];
    uint32_t m_answered = 0, m_dropped = 0;
};
