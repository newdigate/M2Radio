// HciIo -- the platform glue the HCI layer needs: raw bytes each way and a
// millisecond clock.  HciTransport implements it over Serial2; the host unit
// tests implement it with a scripted fake.  MIT.
#pragma once
#include <stdint.h>
#include <stddef.h>

struct HciIo {
    virtual ~HciIo() {}
    virtual size_t   write(const uint8_t *p, size_t n) = 0;
    virtual int      available() = 0;
    virtual int      read() = 0;               // -1 when nothing is waiting
    virtual uint32_t nowMs() = 0;
};
