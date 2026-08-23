// HciEvents -- pure parsers for the HCI events BT-1 consumes.  No I/O, no
// Arduino; host-tested.  Byte layouts from Core 5.2 Vol 4 Part E 7.7.
// MIT, clean-room.
#pragma once
#include <stdint.h>
#include <stddef.h>

struct HciInquiryResult {
    uint8_t  bd[6];          // as on the wire (little-endian); hciFormatBd prints it MSB-first
    uint8_t  psrm;           // Page_Scan_Repetition_Mode
    uint32_t cod;            // Class_Of_Device, 24 bits
    uint16_t clockOffset;
};

struct HciRemoteName {
    uint8_t status;
    uint8_t bd[6];
    char    name[249];       // 248 bytes max plus a terminator we add
};

// Inquiry Result (event 0x02) is FIELD-MAJOR: all BD_ADDRs, then all PSRMs,
// then reserved, then all CoDs, then all clock offsets.  Returns the number
// of complete responses the parameters actually hold (0 when truncated).
uint8_t hciInquiryResultCount(const uint8_t *params, uint8_t len);
bool    hciParseInquiryResult(const uint8_t *params, uint8_t len, uint8_t idx, HciInquiryResult *out);

// Remote Name Request Complete (event 0x07): status(1) bd(6) name(248).
bool    hciParseRemoteNameComplete(const uint8_t *params, uint8_t len, HciRemoteName *out);

// "11:22:33:44:55:66" into a caller buffer of at least 18 bytes.
void    hciFormatBd(const uint8_t bd[6], char out[18]);
