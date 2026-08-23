#include "HciTransport.h"
#include <Arduino.h>
#include <HardwareSerial.h>

void HciTransport::begin(uint32_t baud) {
    m_port.begin(baud);
    m_port.addMemoryForRead(m_rxExtra, sizeof m_rxExtra);
}
// Hand the port back its built-in ring BEFORE dropping the extension: the
// LPUART ISR writes through that pointer, and m_rxExtra dies with this object.
// Harmless for the static instances this library expects, fatal for a stack or
// heap one -- the ISR would keep writing into freed memory after destruction.
void HciTransport::end() {
    m_port.addMemoryForRead(nullptr, 0);
    m_port.end();
}
size_t   HciTransport::write(const uint8_t *p, size_t n) { return m_port.write(p, n); }
int      HciTransport::available() { return m_port.available(); }
int      HciTransport::read()      { return m_port.read(); }
uint32_t HciTransport::nowMs()     { return millis(); }
