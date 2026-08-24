#include "BtFwLoader.h"
#include <string.h>

const char *BtFwLoader::errorName(Error e) {
    switch (e) {
        case OK:         return "ok";
        case NO_IMAGE:   return "no_image";
        case NO_START:   return "no_start_indication";
        case BAD_HEADER: return "bad_header";
        case BAD_CRC:    return "bad_crc";
        case BAD_OFFSET: return "bad_offset";
        case CARD_ERROR: return "card_error";
        case STALLED:    return "stalled";
    }
    return "unknown";
}

uint8_t BtFwLoader::crc8(const uint8_t *p, uint32_t n) {
    uint8_t crc = 0xFF;                       // init 0xFF, polynomial 0x07
    for (uint32_t i = 0; i < n; i++) {
        crc ^= p[i];
        for (uint8_t b = 0; b < 8; b++)
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
    }
    return crc;
}

void BtFwLoader::reset() {
    m_startInds = m_chunks = m_bytesSent = m_retransmits = m_crcErrors = m_maxOffset = 0;
    m_lastCardErr = 0; m_lastOffset = 0; m_haveLast = false;
    m_tFirstN = 0; m_tLastN = 0; m_tLastHead = 0;
}

void BtFwLoader::sendAck() {
    uint8_t a[2] = { ACK, 0 };
    a[1] = crc8(a, 1);                        // crc8 over the header byte alone -> 0x92
    m_io.write(a, 2);
}

// Handle one complete, CRC-checked frame.  *sentAll is set once the image's
// last byte has been served at least once.
BtFwLoader::Error BtFwLoader::onFrame(const uint8_t *f, uint32_t n, bool *sentAll) {
    if (f[0] == START_IND) {
        // AB <chipId:2 LE> <loaderVer:1> <crc8:1>
        m_chipId    = (uint16_t)(f[1] | (f[2] << 8));
        m_loaderVer = f[3];
        m_startInds++;
        sendAck();
        return OK;
    }
    // A7 <len:2 LE> <offset:4 LE> <error:2 LE> <crc8:1>
    uint16_t len   = (uint16_t)(f[1] | (f[2] << 8));
    uint32_t off   = (uint32_t)f[3] | ((uint32_t)f[4] << 8) | ((uint32_t)f[5] << 16) | ((uint32_t)f[6] << 24);
    uint16_t cerr  = (uint16_t)(f[7] | (f[8] << 8));
    (void)n;

    if (cerr) {
        m_lastCardErr = cerr;
        // The card reports what went wrong with the PREVIOUS block and asks
        // again.  Re-serving the requested offset is the recovery, so this is
        // only fatal if it keeps happening -- which the caller's overall
        // timeout bounds.  Bits are recorded rather than interpreted: this
        // driver has never seen a non-zero one on silicon.
    }

    if (len == 0) {                            // nothing wanted; just acknowledge
        sendAck();
        return OK;
    }
    if (off > m_imgLen || (uint32_t)(off + len) > m_imgLen) {
        // Asking past the end is a real fault, not something to paper over by
        // zero-padding: it would silently feed the card rubbish it would then
        // fail to authenticate, and the failure would surface far from here.
        return BAD_OFFSET;
    }

    if (m_tFirstN < TRACE_N) { m_tFirstLen[m_tFirstN] = len; m_tFirstOff[m_tFirstN] = off; m_tFirstN++; }
    m_tLastLen[m_tLastHead] = len; m_tLastOff[m_tLastHead] = off;
    m_tLastHead = (uint8_t)((m_tLastHead + 1) % TRACE_N);
    if (m_tLastN < TRACE_N) m_tLastN++;

    if (m_haveLast && off == m_lastOffset) m_retransmits++;
    m_lastOffset = off; m_haveLast = true;

    sendAck();
    m_io.write(m_img + off, len);
    m_chunks++;
    m_bytesSent += len;
    if (off + len > m_maxOffset) m_maxOffset = off + len;
    if (m_maxOffset >= m_imgLen) *sentAll = true;
    return OK;
}

BtFwLoader::Error BtFwLoader::run(uint32_t startTimeoutMs, uint32_t quietMs,
                                  uint32_t overallMs, void (*idle)()) {
    if (!m_img || !m_imgLen) return NO_IMAGE;
    reset();

    const uint32_t t0 = m_io.nowMs();
    uint32_t lastRx = t0;
    bool sentAll = false;

    uint8_t  frame[10];
    uint32_t have = 0, need = 0;               // need = 0 means "waiting for a header"

    for (;;) {
        uint32_t now = m_io.nowMs();

        while (m_io.available() > 0) {
            int c = m_io.read();
            if (c < 0) break;
            lastRx = now;

            if (need == 0) {                   // header byte
                if ((uint8_t)c == START_IND)      { need = 5; }
                else if ((uint8_t)c == DATA_REQ)  { need = 10; }
                else {
                    // A byte where a header was due.  The card's very first
                    // burst is preceded by a pad-settle 0x00 on this board, so
                    // that one specific value is skipped rather than treated as
                    // a fault -- anything else is a real framing error.
                    if ((uint8_t)c == 0x00) continue;
                    return BAD_HEADER;
                }
                have = 0; frame[have++] = (uint8_t)c;
                continue;
            }

            frame[have++] = (uint8_t)c;
            if (have < need) continue;

            if (crc8(frame, need - 1) != frame[need - 1]) {
                m_crcErrors++;
                have = 0; need = 0;
                continue;                      // let the card retry; it will
            }
            Error e = onFrame(frame, need, &sentAll);
            have = 0; need = 0;
            if (e != OK) return e;
            now = m_io.nowMs();
            lastRx = now;
        }

        if (sentAll && (now - lastRx) >= quietMs) return OK;
        if (!m_startInds && (now - t0) >= startTimeoutMs) return NO_START;
        if ((now - t0) >= overallMs) return m_startInds ? STALLED : NO_START;
        if (idle) idle();
    }
}
