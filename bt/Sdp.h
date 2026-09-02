// Sdp -- the one client query BT-3 needs: the AudioSink service's
// ProtocolDescriptorList, from which the AVDTP version is read.  Clean-room
// from the SDP data-element grammar (Core Vol 3 Part B).  MIT.
#pragma once
#include <stdint.h>
struct Sdp {
    static const uint16_t PSM = 0x0001;
    // ServiceSearchAttributeRequest: DES{UUID16 0x110B}, max 1008 bytes, DES{UINT16 attr 0x0004}, no continuation
    static uint16_t buildAudioSinkPdlRequest(uint8_t *out, uint16_t txn);
    // Scans a ServiceSearchAttributeResponse for [UUID16 0x0019][UINT16 version]; 0 if absent
    static uint16_t parseAvdtpVersion(const uint8_t *rsp, uint16_t len);
};
