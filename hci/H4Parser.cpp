#include "H4Parser.h"

H4Parser::H4Parser()
    : m_aclMax(ACL_MAX_DEFAULT), m_packets(0), m_faults(0),
      m_onPacket(nullptr), m_onFault(nullptr), m_ctx(nullptr) {
    reset();
}

void H4Parser::setCallbacks(PacketFn onPacket, FaultFn onFault, void *ctx) {
    m_onPacket = onPacket; m_onFault = onFault; m_ctx = ctx;
}

void H4Parser::setAclMax(uint16_t max) {
    const uint16_t cap = (uint16_t)(MAX_PACKET - ACL_HDR);
    m_aclMax = max > cap ? cap : max;
}

void H4Parser::reset() {
    m_state = WAIT_TYPE; m_type = 0; m_len = 0; m_need = 0; m_hdrLen = 0;
}

void H4Parser::feed(const uint8_t *p, size_t n) {
    for (size_t i = 0; i < n; i++) feed(p[i]);
}

void H4Parser::fault(uint8_t f, uint8_t b) {
    m_faults++;
    reset();
    if (m_onFault) m_onFault(m_ctx, f, b);
}

void H4Parser::emit() {
    m_packets++;
    if (m_onPacket) m_onPacket(m_ctx, m_type, m_buf, m_len);
    reset();
}

void H4Parser::feed(uint8_t b) {
    switch (m_state) {
    case WAIT_TYPE:
        if (b == EVENT)    { m_type = b; m_hdrLen = EVT_HDR; m_state = HEADER; }
        else if (b == ACL) { m_type = b; m_hdrLen = ACL_HDR; m_state = HEADER; }
        else fault(BAD_TYPE, b);
        return;
    case HEADER:
        m_buf[m_len++] = b;
        if (m_len < m_hdrLen) return;
        if (m_type == EVENT) {
            m_need = EVT_HDR + m_buf[1];
        } else {
            uint16_t dlen = (uint16_t)(m_buf[2] | (m_buf[3] << 8));
            if (dlen > m_aclMax) { fault(BAD_LENGTH, b); return; }
            m_need = ACL_HDR + dlen;
        }
        if (m_len == m_need) emit(); else m_state = PAYLOAD;
        return;
    case PAYLOAD:
        m_buf[m_len++] = b;
        if (m_len == m_need) emit();
        return;
    }
}
