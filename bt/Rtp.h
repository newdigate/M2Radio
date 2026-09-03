// Rtp -- RTP v2 header (RFC 3550) + the A2DP v1.3 SBC media payload header.
// Pure framing: no I/O, no state.  MIT, clean-room from the specs.
#pragma once
#include <stdint.h>
struct Rtp {
    static const uint8_t  PAYLOAD_TYPE = 96;        // dynamic PT used for A2DP media
    static const uint32_t SSRC = 0x00000001u;       // fixed per build; the sink does not care
    static const uint16_t HEADER_LEN = 13;          // 12-byte RTP header + 1-byte A2DP payload header
    // Write HEADER_LEN bytes into out: the 12-byte RTP v2 header (big-endian seq,
    // timestamp, SSRC; V=2, no padding/extension/CSRC, marker clear, PT=96) then the
    // 1-byte SBC media payload header (A2DP v1.3 sec 4.3.4): not fragmented (F=S=L=0),
    // frameCount in the low nibble.  Returns HEADER_LEN.
    static uint16_t header(uint8_t *out, uint16_t seq, uint32_t timestamp, uint8_t frameCount);
};
