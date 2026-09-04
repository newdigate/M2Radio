// Sdp -- the CLIENT query BT-3 needs (the AudioSink's ProtocolDescriptorList,
// from which the AVDTP version is read) and the SERVER half a sink needs from
// us: one AudioSource (0x110A) record.  Both headsets SDP-query the source on
// AVDTP contact (BT-2 transcript 2026-08-29), and in the Mac->Shokz reference
// the Shokz asks the source's A2DP profile version (attribute 0x0009) and
// answers DISCOVER only after that query completes -- so an unanswered query
// is a plausible DISCOVER blocker.  Clean-room from the SDP data-element
// grammar and PDU formats (Core Vol 3 Part B).  MIT, pure functions.
#pragma once
#include <stdint.h>
struct Sdp {
    static const uint16_t PSM = 0x0001;
    // ServiceSearchAttributeRequest: DES{UUID16 0x110B}, max 1008 bytes, DES{UINT16 attr 0x0004}, no continuation
    static uint16_t buildAudioSinkPdlRequest(uint8_t *out, uint16_t txn);
    // Scans a ServiceSearchAttributeResponse for [UUID16 0x0019][UINT16 version]; 0 if absent
    static uint16_t parseAvdtpVersion(const uint8_t *rsp, uint16_t len);
    // --- server: answer one SDP request PDU against our AudioSource record ---
    // Handles ServiceSearch (0x02), ServiceAttribute (0x04) and ServiceSearchAttribute (0x06);
    // anything else, or a malformed request, gets an ErrorResponse.  Attribute lists larger
    // than the peer's MaxAttributeByteCount or its L2CAP MTU (`mtu`) go out in chunks through
    // a 2-byte continuation state.  Returns the response length (0 only if there is not even
    // a transaction id to answer).  Never writes past outMax.
    static uint16_t serve(const uint8_t *req, uint16_t len, uint16_t mtu, uint8_t *out, uint16_t outMax);
    static const uint32_t RECORD_HANDLE = 0x00010000;
};
