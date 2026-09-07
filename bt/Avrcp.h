// Avrcp -- the MINIMAL AVRCP target (NEW-34 piece 3): AVCTP (PSM 0x0017) framing plus the one AV/C
// command a real headset sends to decide the link is live.  Measured on a Shokz OpenMove 2026-09-07:
// once we serve an AVRCP Target SDP record the headset sends exactly one AV/C command --
// GetCapabilities(EVENTS_SUPPORTED) then RegisterNotification(PLAYBACK_STATUS_CHANGED) -- and waits.  We answer
// STABLE {PLAYBACK_STATUS_CHANGED} and INTERIM + PLAYING.  Every
// other AV/C command gets AV/C's NOT IMPLEMENTED response (ctype 0x08, operands echoed), so a peer never
// hangs on an unanswered transaction.  Sized to the capture: nothing the headset has not sent is built.
// RX entry point only RECORDS (one command slot -- AVCTP transactions are sequential per channel; a
// second command landing before service() ran replaces the first, counted in dropped()); the reply is
// queued from service() -- writing to the transport from the RX pump bus-faults (B6, 2026-08-28).
// MIT, clean-room from the AVCTP 1.4 / AV/C / AVRCP 1.4 specifications, no heap.
#pragma once
#include <stdint.h>
#include "L2cap.h"
class Avrcp {
public:
    static const uint16_t PSM = 0x0017;
    static const uint16_t PID = 0x110E;                 // AVCTP profile id: AV Remote Control
    static const uint8_t  MAX_CMD = 64;
    // From the L2cap data callback: true when the payload was an AVCTP command on a PSM-0x0017 channel
    // (recorded here, answered from service()); false for anything else.
    bool onData(const L2cap::Channel &ch, const uint8_t *p, uint16_t len);
    void service(L2cap &l2);                            // main context: build + queue the pending reply (retried while the TXQ is full)
    void reset() { m_pending = false; m_cid = 0; m_len = 0; }
    bool     pending()       const { return m_pending; }
    uint32_t notifications() const { return m_notifications; }   // RegisterNotification(PLAYBACK_STATUS_CHANGED) answered INTERIM PLAYING
    uint32_t unsupported()   const { return m_unsupported; }     // AV/C commands answered NOT IMPLEMENTED (GetCapabilities is answered STABLE and counted here too, as "not a notification")
    uint32_t dropped()       const { return m_dropped; }
    // Build the AV/C response for one AVCTP command frame (pure; host-tested).  Returns the response
    // length (0 = not an AVRCP command: wrong PID, fragment, or a response frame -- ignored).
    static uint16_t respond(const uint8_t *cmd, uint16_t len, uint8_t *out, uint16_t outMax, bool *wasNotification);
private:
    volatile bool m_pending = false; uint16_t m_cid = 0, m_len = 0; uint8_t m_cmd[MAX_CMD];
    uint32_t m_notifications = 0, m_unsupported = 0, m_dropped = 0;
};
