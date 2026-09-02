// HciTransport -- HciIo over a core HardwareSerialIMXRT port (Serial2 = LPUART2
// = the M.2 socket's BT HCI UART on the MIMXRT1170-EVKB).  Adds a 1 KB RX
// ring on top of the core's 64-byte one: an Extended Inquiry Result event is
// 257 bytes and LPUART2 has no flow control on this board (RTS is the gigabit
// PHY's reset line), so the ring is the only slack there is.
// MIT.
#pragma once
#include <stdint.h>
#include <stddef.h>
#include "HciIo.h"

class HardwareSerialIMXRT;

class HciTransport : public HciIo {
public:
    static const size_t RX_EXTRA = 1024;
    explicit HciTransport(HardwareSerialIMXRT &port) : m_port(port) {}
    void begin(uint32_t baud);
    void end();
    // Re-program the port at a new rate.  Goes through end() so the core
    // disables the transmitter/receiver and drains any in-flight TX before the
    // BAUD divider is rewritten, then begin() re-attaches the RX extension.
    // Call ONLY after the controller has acknowledged its own rate change and
    // the line is idle: the ring's buffered bytes are discarded, by design.
    void rebaud(uint32_t baud) { end(); begin(baud); }
    size_t   write(const uint8_t *p, size_t n) override;
    int      available() override;
    int      read() override;
    uint32_t nowMs() override;
private:
    HardwareSerialIMXRT &m_port;
    uint8_t m_rxExtra[RX_EXTRA];
};
