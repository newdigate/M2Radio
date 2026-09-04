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
    // TX extension (NEW-33, 2026-09-04).  The core's Serial2 TX ring is 64 B and
    // HardwareSerialIMXRT::write() spins in yield() when it is full, so every
    // ~620 B ACL media packet blocked the caller ~1.9 ms at 3 Mbaud -- measured
    // in acid_box's main loop: `svc` tracked `txb x 3.33 us` at ~130 ms/s while
    // streaming (the NEW-33 transcript).  Sized PAST THE ACL CREDIT WINDOW:
    // L2cap writes a packet only while it holds a controller credit, and credits
    // return only after the air transmission, so bytes resident in this ring can
    // never exceed aclNum x (9 + L2cap::MAX_PAYLOAD) = 7 x 709 = 4963 B on the
    // IW416 (hci_buffer acl_num=7).  The UART drains at 3 Mbaud, far faster than
    // the air link, so the credits -- not this ring -- are the bottleneck, and
    // write() never blocks.  hci/ cannot see bt/'s MAX_PAYLOAD, so the consumer
    // that knows both pins the bound with a static_assert (acid_box does) and
    // prints the measured ring against the real aclNum at connect.
    static const size_t TX_EXTRA = 5120;
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
    uint8_t m_txExtra[TX_EXTRA];
};
