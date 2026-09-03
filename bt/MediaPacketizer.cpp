#include "MediaPacketizer.h"
#include <string.h>
// SPSC ring, one slot sacrificed: wr==rd is EMPTY, (wr+1)%RING==rd is FULL, so RING=65
// gives 64 usable frames.  Only push() writes m_wr (and, on overflow, advances m_rd to
// drop the oldest); only drain() writes m_rd on a successful send.  count() is read-only.
uint8_t MediaPacketizer::count() const {
    return (uint8_t)(((int)m_wr - (int)m_rd + RING) % RING);
}
void MediaPacketizer::begin(uint16_t mtu) {
    m_mtu = mtu;
    uint16_t avail = mtu > Rtp::HEADER_LEN ? mtu - Rtp::HEADER_LEN : 0;
    m_perPkt = avail / 119;                 // whole SBC frames (bitpool 53 = 119 B) per packet
    if (m_perPkt == 0) m_perPkt = 1;
    if (m_perPkt > 8)  m_perPkt = 8;        // A2DP frame-count nibble cap; and PKT_MAX sizing
    m_wr = m_rd = 0; m_seq = 0; m_ts = 0;
    m_frames = m_packets = m_drops = 0; m_hw = 0;
}
void MediaPacketizer::push(const uint8_t *frame, uint16_t len) {
    if (len > FRAME_MAX) len = FRAME_MAX;
    uint8_t nextWr = (uint8_t)((m_wr + 1) % RING);
    if (nextWr == m_rd) {                    // ring full -> drop OLDEST
        m_rd = (uint8_t)((m_rd + 1) % RING);
        m_drops++;
    }
    memcpy(m_buf[m_wr], frame, len); m_len[m_wr] = len;
    m_wr = nextWr;
    m_frames++;
    uint8_t q = count();
    if (q > m_hw) m_hw = q;
}
void MediaPacketizer::drain(SendFn send, void *ctx) {
    uint8_t pkt[PKT_MAX];
    while (m_wr != m_rd) {                   // frames available
        uint8_t rd = m_rd, n = 0; uint16_t off = Rtp::HEADER_LEN;
        // gather up to m_perPkt whole frames that still fit the MTU
        while (n < m_perPkt && rd != m_wr && off + m_len[rd] <= m_mtu) {
            memcpy(pkt + off, m_buf[rd], m_len[rd]); off += m_len[rd];
            rd = (uint8_t)((rd + 1) % RING); n++;
        }
        if (n == 0) break;                   // defensive: a frame larger than the MTU (cannot happen at bitpool 53)
        Rtp::header(pkt, m_seq, m_ts, n);
        if (!send(ctx, pkt, off)) return;    // sink refused -> keep frames, try next drain()
        m_rd = rd; m_seq++; m_ts += (uint32_t)n * 128; m_packets++;
    }
}
