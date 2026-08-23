#include "HciTransport.h"
#include <Arduino.h>
#include <HardwareSerial.h>

void HciTransport::begin(uint32_t baud) {
    m_port.begin(baud);
    m_port.addMemoryForRead(m_rxExtra, sizeof m_rxExtra);
}
void HciTransport::end() { m_port.end(); }
size_t   HciTransport::write(const uint8_t *p, size_t n) { return m_port.write(p, n); }
int      HciTransport::available() { return m_port.available(); }
int      HciTransport::read()      { return m_port.read(); }
uint32_t HciTransport::nowMs()     { return millis(); }
