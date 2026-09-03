#include "MediaPacketizer.h"
#include <string.h>
// PRIMASK save/restore critical section: serialises drain()'s m_rd commit against
// push() running in the audio ISR.  ARM-only; a no-op on the host build so the
// unit tests stay pure.
#if defined(__arm__) || defined(__ARM_ARCH)
static inline uint32_t mp_irq_save() { uint32_t p; __asm volatile ("mrs %0, primask\n\tcpsid i" : "=r"(p) :: "memory"); return p; }
static inline void     mp_irq_restore(uint32_t p) { __asm volatile ("msr primask, %0" :: "r"(p) : "memory"); }
#else
static inline uint32_t mp_irq_save() { return 0; }
static inline void     mp_irq_restore(uint32_t) {}
#endif

// drain() wants m_rd = rd0 + n.  If push() (ISR) advanced m_rd past that while we
// were sending (it dropped >= n oldest frames), keep the ISR's further position;
// never move m_rd backward.  All arithmetic is mod RING, so it is wrap-safe.
uint8_t MediaPacketizer::advanceRd(uint8_t cur, uint8_t rd0, uint8_t n) {
    uint8_t moved = (uint8_t)(((int)cur - (int)rd0 + RING) % RING);
    if (moved >= n) return cur;
    return (uint8_t)(((int)rd0 + n) % RING);
}
// SPSC-ish ring, one slot sacrificed: wr==rd is EMPTY, (wr+1)%RING==rd is FULL,
// so RING=65 gives 64 usable frames.  push() (producer/audio ISR) writes m_wr, and
// ALSO advances m_rd when it drops the oldest on a full ring; drain() (consumer/main
// loop) advances m_rd on a successful send.  m_rd therefore has TWO writers.
// CLOSED: drain()'s commit no longer clobbers a concurrent drop.  It captures rd0
// (the index it started gathering from) before sending, then commits through
// advanceRd(m_rd, rd0, n) inside a PRIMASK critical section (mp_irq_save/restore
// above -- __disable_irq/__enable_irq on ARM, a no-op on host) so push()'s
// drop-oldest write can't interleave with the read-modify-write.  advanceRd() only
// ever moves m_rd forward: if the ISR has already advanced it past rd0+n (it
// dropped >= n oldest frames while we were sending), the ISR's further position is
// kept rather than being rebased backward.  So drops==0 (the phase-4 target) stays
// sound rather than merely quiet, and the drops>0 backpressure regime -- previously
// a benign race that could resurface a sacrificed slot or double-send a
// drop-counted frame -- is race-free too.  Byte loads/stores of m_wr/m_rd are
// single-copy-atomic on Cortex-M7, so no torn reads and no barrier is needed beyond
// the critical section.  count() below only reads m_wr/m_rd, never writes either.
uint8_t MediaPacketizer::count() const {
    return (uint8_t)(((int)m_wr - (int)m_rd + RING) % RING);
}
// Assumes the fixed SBC config named at FRAME_BYTES's definition -- a different
// bitpool/subband/block config would need a different frame size here.
void MediaPacketizer::begin(uint16_t mtu) {
    m_mtu = mtu;
    uint16_t avail = mtu > Rtp::HEADER_LEN ? mtu - Rtp::HEADER_LEN : 0;
    m_perPkt = avail / FRAME_BYTES;          // whole SBC frames per packet
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
        uint8_t rd0 = m_rd, rd = rd0, n = 0; uint16_t off = Rtp::HEADER_LEN;
        // gather up to m_perPkt whole frames that still fit the MTU
        while (n < m_perPkt && rd != m_wr && off + m_len[rd] <= m_mtu) {
            memcpy(pkt + off, m_buf[rd], m_len[rd]); off += m_len[rd];
            rd = (uint8_t)((rd + 1) % RING); n++;
        }
        if (n == 0) break;                   // defensive: a frame larger than the MTU (cannot happen at bitpool 53)
        Rtp::header(pkt, m_seq, m_ts, n);
        if (!send(ctx, pkt, off)) return;    // sink refused -> keep frames, try next drain()
        // Commit: advance m_rd forward to rd0+n without clobbering a concurrent
        // ISR drop (which may have already advanced m_rd).
        uint32_t s = mp_irq_save();
        m_rd = advanceRd(m_rd, rd0, n);
        mp_irq_restore(s);
        m_seq++; m_ts += (uint32_t)n * 128; m_packets++;
    }
}
