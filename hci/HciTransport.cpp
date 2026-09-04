#include "HciTransport.h"
#include <Arduino.h>
#include <HardwareSerial.h>

void HciTransport::begin(uint32_t baud) {
    m_port.begin(baud);
    m_port.addMemoryForRead(m_rxExtra, sizeof m_rxExtra);
    m_port.addMemoryForWrite(m_txExtra, sizeof m_txExtra);   // credit-bounded: never fills (header)
}
// Hand the port back its built-in rings BEFORE dropping the extensions: the
// LPUART ISR writes/reads through those pointers, and both extensions die with
// this object.  Harmless for the static instances this library expects, fatal
// for a stack or heap one -- the ISR would keep using freed memory after
// destruction.  ORDER MATTERS FOR TX: addMemoryForWrite() resets head/tail,
// which would DROP bytes still queued (rebaud()'s vendor set-baud command
// sits in this ring when end() runs), so the port is end()ed FIRST -- the
// core's end() waits for the transmitter to drain -- and only then is the TX
// extension released.  RX is released first, as before: nothing is lost
// there, the bytes merely land in the built-in ring.
void HciTransport::end() {
    m_port.addMemoryForRead(nullptr, 0);
    m_port.end();                            // drains TX (waits for transmitting_ to clear)
    m_port.addMemoryForWrite(nullptr, 0);    // AFTER the drain
}
size_t   HciTransport::write(const uint8_t *p, size_t n) { return m_port.write(p, n); }
int      HciTransport::available() { return m_port.available(); }
int      HciTransport::read()      { return m_port.read(); }
uint32_t HciTransport::nowMs()     { return millis(); }
