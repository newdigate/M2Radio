// Avrcp -- the MINIMAL AVRCP target (NEW-34 piece 3): AVCTP (PSM 0x0017) framing plus the one AV/C
// command a real headset sends to decide the link is live.  Measured on a Shokz OpenMove 2026-09-07:
// once we serve an AVRCP Target SDP record the headset sends exactly one AV/C command --
// GetCapabilities(EVENTS_SUPPORTED) then RegisterNotification(PLAYBACK_STATUS_CHANGED) -- and waits.  We answer
// STABLE {PLAYBACK_STATUS_CHANGED, VOLUME_CHANGED} and INTERIM + PLAYING.  Every
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
    void reset() { m_pending = false; m_cid = 0; m_len = 0; m_volRegistered = false; m_volLabel = 0; m_volChanged = false; }
    bool     pending()       const { return m_pending; }
    uint32_t notifications() const { return m_notifications; }   // RegisterNotification answered INTERIM (PLAYBACK_STATUS_CHANGED -> PLAYING, VOLUME_CHANGED -> the current volume)
    // WHAT each AV/C command was answered with, as three counters (NEW-42 -- one bool used to fold GetCapabilities
    // and SetAbsoluteVolume, both answered properly, into unsupported(), so the sink's heartbeat read a phone's
    // volume presses as commands we do not implement):
    uint32_t answered()      const { return m_answered; }        // answered from respond() with a real reply: GetCapabilities, SetAbsoluteVolume
    uint32_t unsupported()   const { return m_unsupported; }     // NOT IMPLEMENTED (unknown PDU), or IPID for a foreign profile id
    enum { KIND_NOT_IMPLEMENTED = 0, KIND_NOTIFICATION = 1, KIND_ANSWERED = 2 };
    uint32_t dropped()       const { return m_dropped; }
    // --- absolute volume (AVRCP 1.4 sec 6.13) --------------------------------------------------------------
    // The target keeps ONE volume, 0..127.  The phone writes it with SetAbsoluteVolume (PDU 0x50), which is
    // ACCEPTED and handed to the callback below (that is what makes iOS treat us as a real speaker); the local
    // side (a knob) writes it with setLocalVolume(), which raises VOLUME_CHANGED once per registration.  A CT
    // registers for VOLUME_CHANGED one notification at a time: we answer INTERIM immediately, send exactly one
    // CHANGED when the local volume next moves, and then wait for the CT to register again.
    typedef void (*VolumeFn)(void *ctx, uint8_t vol0_127);
    // Applied whenever the peer sets the absolute volume.  A file-scope hook, not a member, because respond()
    // is static by design (pure, host-tested against byte vectors) and applies the volume as it builds the reply.
    static void setVolumeCallback(VolumeFn fn, void *ctx);
    void    setLocalVolume(uint8_t v);                  // the local side moved the volume: update + raise CHANGED if registered
    uint8_t volume() const { return m_vol; }
    // Build the AV/C response for one AVCTP command frame (pure; host-tested).  Returns the response
    // length (0 = not an AVRCP command: wrong PID, fragment, or a response frame -- ignored).  kind (optional)
    // receives KIND_NOTIFICATION / KIND_ANSWERED / KIND_NOT_IMPLEMENTED for what was built; volumeSet (optional)
    // receives the volume a SetAbsoluteVolume applied, so the OBJECT can track what respond() did; it is left
    // untouched for every other command.
    static uint16_t respond(const uint8_t *cmd, uint16_t len, uint8_t *out, uint16_t outMax, uint8_t *kind,
                            uint8_t *volumeSet = nullptr);
private:
    // True when m_cmd is a RegisterNotification for VOLUME_CHANGED -- answered by the object, not by respond(),
    // because the INTERIM carries live state (m_vol) and arms the one-shot CHANGED.
    bool isVolumeRegistration() const;
    volatile bool m_pending = false; uint16_t m_cid = 0, m_len = 0; uint8_t m_cmd[MAX_CMD];
    uint32_t m_notifications = 0, m_answered = 0, m_unsupported = 0, m_dropped = 0;
    uint8_t m_vol = 100; bool m_volRegistered = false; uint8_t m_volLabel = 0; bool m_volChanged = false;
};
