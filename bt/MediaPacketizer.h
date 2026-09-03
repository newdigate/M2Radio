// MediaPacketizer -- the A2DP media send core: a fixed SPSC ring of whole SBC
// frames (producer = audio ISR, consumer = main loop), drop-OLDEST on overflow,
// and RTP-batched draining up to the media MTU.  No AudioStream, no L2cap, no
// Arduino: pure logic so it host-compiles and is tested like Sbc/L2cap/Avdtp.
// MIT.
#pragma once
#include <stdint.h>
#include "Rtp.h"
class MediaPacketizer {
public:
    static const uint8_t  RING = 65;        // 64 USABLE frames (~190 ms at 344 fps); one slot
                                            // is sacrificed so wr==rd means EMPTY and nextWr==rd
                                            // means FULL -- a lock-free SPSC ring with no count field.
    static const uint16_t FRAME_MAX = 128;  // an SBC frame is 119 B at bitpool 53; round up
    // One SBC frame at 44.1k/joint-stereo/16-block/8-subband/bitpool-53 (the fixed encoder
    // config `begin()` assumes when computing frames-per-packet).
    static const uint16_t FRAME_BYTES = 119;
    static const uint16_t PKT_MAX = Rtp::HEADER_LEN + 8 * FRAME_MAX;
    // send returns false when the sink is not ready (no L2CAP credit); the packetiser
    // keeps the frames for the next drain().
    typedef bool (*SendFn)(void *ctx, const uint8_t *pkt, uint16_t len);

    void begin(uint16_t mtu);
    // Producer side (call from the ISR).  Copies the frame; drops the OLDEST on a full ring.
    void push(const uint8_t *frame, uint16_t len);
    // Consumer side (call from the main loop).  Batches whole frames up to mtu into RTP
    // packets and sends each via SendFn until the ring is empty or SendFn refuses.
    void drain(SendFn send, void *ctx);

    uint32_t frames()        const { return m_frames; }
    uint32_t packets()       const { return m_packets; }
    uint32_t drops()         const { return m_drops; }
    uint8_t  queueHighWater() const { return m_hw; }
private:
    uint8_t  count() const;                 // frames currently queued
    uint8_t  m_buf[RING][FRAME_MAX]; uint16_t m_len[RING];
    volatile uint8_t m_wr = 0, m_rd = 0;    // SPSC indices, mod RING
    uint16_t m_mtu = 0, m_perPkt = 0;
    uint16_t m_seq = 0; uint32_t m_ts = 0;
    uint32_t m_frames = 0, m_packets = 0, m_drops = 0; uint8_t m_hw = 0;
};
