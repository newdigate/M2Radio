// BtFwLoader -- NXP "V3" Bluetooth firmware download over the HCI UART.
//
// WHY THIS EXISTS.  On the MIMXRT1170-EVKB's M2-MAYA-W161 (NXP IW416) the
// combo blob downloaded over SDIO brings up the WLAN core and NOT the BT core:
// measured 2026-08-23, the SDIO download stops 8,776 bytes short of the image
// (sent=402288/411064, last_req=0) and the BT core never answers an HCI
// command.  What it DOES do is announce itself on LPUART2 and wait for a
// firmware download of its own.  This is that download.
//
// THE PROTOCOL, as decoded from the card's own bytes (see below) and confirmed
// against NXP's fw_loader_uart.c constants -- facts only, nothing transcribed;
// that file is NXP LA_OPT licensed and this implementation is clean-room:
//
//   card -> host   START INDICATION   AB <chipId:2 LE> <loaderVer:1> <crc8:1>
//   card -> host   DATA REQUEST       A7 <len:2 LE> <offset:4 LE> <error:2 LE> <crc8:1>
//   host -> card   ACK                7A <crc8:1>
//   host -> card   <len bytes of the image, taken from `offset`>
//
// Every frame's CRC-8 covers the WHOLE frame INCLUDING its header byte:
// polynomial 0x07, init 0xFF, no reflection, no final xor.
//
// ★ THAT DECODE IS NOT A GUESS -- it was verified three independent ways
// against a real card's 16-byte power-up burst
// (00 AB017200 47 AB017200 47 AB017200 47, the leading 00 being the usual
// pad-settle artefact):
//   * 0xAB is NXP's V3 start-indication marker;
//   * the chipId it carries, 0x7201, is the SAME hw_version this repo reads
//     independently over SDIO with GET_HW_SPEC -- two interfaces sharing no
//     code agreeing on one number;
//   * crc8(AB 01 72 00) with the parameters above is 0x47, exactly the byte
//     the card sent.
//
// The card repeats the start indication a few times and then goes silent, so a
// host that never answers sees precisely what this repo saw for six days: a
// healthy card that will not speak Bluetooth.
//
// MIT.  Clean-room from the wire and from published constants.
#pragma once
#include <stdint.h>
#include <stddef.h>
#include "HciIo.h"

class BtFwLoader {
public:
    enum Error : uint8_t {
        OK = 0,
        NO_IMAGE,     // setImage() was never called
        NO_START,     // the card never sent a start indication
        BAD_HEADER,   // a byte arrived where a header was due and it was neither AB nor A7
        BAD_CRC,      // a frame's CRC-8 did not match
        BAD_OFFSET,   // the card asked for bytes outside the image
        CARD_ERROR,   // the card reported an error it did not recover from
        STALLED,      // the card stopped asking before the whole image was sent
    };
    static const char *errorName(Error e);

    static const uint8_t START_IND = 0xAB;   // + 4 bytes
    static const uint8_t DATA_REQ  = 0xA7;   // + 9 bytes
    static const uint8_t ACK       = 0x7A;   // + 1 byte (its own crc8)

    // CRC-8, polynomial 0x07, init 0xFF, over the bytes as given.
    static uint8_t crc8(const uint8_t *p, uint32_t n);

    explicit BtFwLoader(HciIo &io) : m_io(io) { reset(); }

    void setImage(const uint8_t *image, uint32_t len) { m_img = image; m_imgLen = len; }

    // Drive the whole download.  Returns when the image has been delivered and
    // the card has gone quiet (OK), or on the first unrecoverable fault.
    //   startTimeoutMs -- how long to wait for the first start indication
    //   quietMs        -- silence after the last byte of the image that counts as done
    //   overallMs      -- hard ceiling on the whole exchange
    //   idle           -- called between polls (e.g. delay(1))
    Error run(uint32_t startTimeoutMs, uint32_t quietMs, uint32_t overallMs, void (*idle)() = nullptr);

    uint16_t chipId()      const { return m_chipId; }
    uint8_t  loaderVer()   const { return m_loaderVer; }
    uint32_t startInds()   const { return m_startInds; }
    uint32_t chunks()      const { return m_chunks; }
    uint32_t bytesSent()   const { return m_bytesSent; }
    uint32_t retransmits() const { return m_retransmits; }
    uint32_t crcErrors()   const { return m_crcErrors; }   // frames WE rejected
    uint16_t lastCardErr() const { return m_lastCardErr; } // error bits the CARD reported
    uint32_t maxOffset()   const { return m_maxOffset; }   // highest offset+len served

    // A small trace of the request sequence, for bring-up.  The first N are
    // where the structure shows (header blocks vs data blocks); the last N are
    // where a download that ends wrongly shows it.  Kept tiny and fixed-size.
    static const uint8_t TRACE_N = 12;
    uint8_t  traceFirstN() const { return m_tFirstN; }
    uint8_t  traceLastN()  const { return m_tLastN; }
    uint16_t traceFirstLen(uint8_t i) const { return m_tFirstLen[i]; }
    uint32_t traceFirstOff(uint8_t i) const { return m_tFirstOff[i]; }
    uint16_t traceLastLen(uint8_t i)  const { return m_tLastLen[i]; }
    uint32_t traceLastOff(uint8_t i)  const { return m_tLastOff[i]; }

private:
    void reset();
    void sendAck();
    Error onFrame(const uint8_t *f, uint32_t n, bool *sentAll);

    HciIo   &m_io;
    const uint8_t *m_img = nullptr;
    uint32_t m_imgLen = 0;
    uint16_t m_chipId = 0;
    uint8_t  m_loaderVer = 0;
    uint32_t m_startInds, m_chunks, m_bytesSent, m_retransmits, m_crcErrors, m_maxOffset;
    uint16_t m_lastCardErr;
    uint32_t m_lastOffset;   // for retransmit detection
    bool     m_haveLast;
    uint16_t m_tFirstLen[TRACE_N]; uint32_t m_tFirstOff[TRACE_N]; uint8_t m_tFirstN;
    uint16_t m_tLastLen[TRACE_N];  uint32_t m_tLastOff[TRACE_N];  uint8_t m_tLastN; uint8_t m_tLastHead;
};
