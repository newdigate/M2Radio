// HciPump -- one bounded Hci::service() per yield(), from a yield-attached
// EventResponder.  The same shape as the Wi-Fi facade's pump
// (WiFiClass::serviceEvent), so the two coexist and every delay() services
// both.  Single instance, like the facade.  MIT.
#pragma once
#include <stdint.h>
#include <Arduino.h>
#include <EventResponder.h>

class Hci;

class HciPump {
public:
    void attach(Hci &hci);
    void detach();
    bool     attached() const { return m_attached; }
    uint32_t passes()   const { return m_passes; }
private:
    static void serviceEvent(EventResponderRef ref);
    static HciPump *s_self;
    EventResponder m_responder;
    Hci     *m_hci = nullptr;
    bool     m_attached = false;
    uint32_t m_passes = 0;
};
