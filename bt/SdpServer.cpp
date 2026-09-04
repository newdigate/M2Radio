#include "SdpServer.h"
#include "Sdp.h"
#include <string.h>
bool SdpServer::onData(const L2cap::Channel &ch, const uint8_t *p, uint16_t len) {
    if (ch.psm != Sdp::PSM || !ch.peerInitiated) return false;
    if (m_pending) m_dropped++;                                   // a request landed before service() answered the last one
    uint16_t n = len > MAX_REQ ? MAX_REQ : len;                   // a truncated copy still draws an ErrorResponse, never silence
    memcpy(m_req, p, n); m_len = n; m_cid = ch.remoteCid; m_mtu = ch.mtuOut;
    m_pending = true;
    return true;
}
void SdpServer::service(L2cap &l2) {
    if (!m_pending) return;
    uint8_t rsp[128]; uint16_t n = Sdp::serve(m_req, m_len, m_mtu, rsp, sizeof rsp);
    if (!n) { m_pending = false; return; }                        // not even a transaction id to answer
    if (l2.send(m_cid, rsp, n)) { m_pending = false; m_answered++; }   // else: TXQ full, retry next tick
}
