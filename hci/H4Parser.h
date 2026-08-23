// H4Parser -- reassembles HCI packets from the H4 (UART) byte stream on the
// HOST side of the link.  Pure state machine: no I/O, no Arduino, so it is
// unit-tested on the host (hci/test/).
//
// A host receives two packet types (Core 5.2 Vol 4 Part A 2):
//   0x04 HCI Event : [type][event_code][param_len][params ...]
//   0x02 ACL data  : [type][handle lo][handle hi][len lo][len hi][data ...]
// (loopback-mode echoes arrive as Loopback Command EVENTS, not as command
// packets; SCO is unused on this board.)  Any other type byte is a framing
// fault.  H4 has no sync marker, so a lost byte desyncs the stream for good.
// This class recovers its OWN state immediately -- fault() has already reset
// it to WAIT_TYPE by the time the callback runs -- but that is not enough on a
// real link: the next bytes are still mid-packet garbage.  The LINK recovery
// policy (discard until the line has been idle a while) belongs to the owner,
// which the fault callback exists to tell.
//
// MIT.  Clean-room from the specification.
#pragma once
#include <stdint.h>
#include <stddef.h>

class H4Parser {
public:
    enum Type  : uint8_t { ACL = 0x02, EVENT = 0x04 };
    enum Fault : uint8_t { BAD_TYPE = 1, BAD_LENGTH = 2 };

    // pkt EXCLUDES the H4 type byte: for EVENT it is [code][plen][params],
    // for ACL it is [handle lo][handle hi][len lo][len hi][data].
    typedef void (*PacketFn)(void *ctx, uint8_t type, const uint8_t *pkt, size_t len);
    typedef void (*FaultFn)(void *ctx, uint8_t fault, uint8_t byte);

    static const size_t   EVT_HDR = 2, ACL_HDR = 4;
    static const uint16_t ACL_MAX_DEFAULT = 1024;        // plausibility bound until Read_Buffer_Size
    static const size_t   MAX_PACKET = ACL_HDR + 1024;   // the largest packet the buffer can hold

    H4Parser();
    void setCallbacks(PacketFn onPacket, FaultFn onFault, void *ctx);
    void setAclMax(uint16_t max);                        // clamped to MAX_PACKET - ACL_HDR
    void feed(uint8_t b);
    void feed(const uint8_t *p, size_t n);
    void reset();                                        // back to waiting for a type byte
    bool     idle()    const { return m_state == WAIT_TYPE; }
    uint32_t packets() const { return m_packets; }
    uint32_t faults()  const { return m_faults; }

private:
    enum State : uint8_t { WAIT_TYPE, HEADER, PAYLOAD };
    void emit();
    void fault(uint8_t f, uint8_t b);

    State    m_state;
    uint8_t  m_type;
    size_t   m_len, m_need, m_hdrLen;
    uint16_t m_aclMax;
    uint32_t m_packets, m_faults;
    uint8_t  m_buf[MAX_PACKET];
    PacketFn m_onPacket; FaultFn m_onFault; void *m_ctx;
};
