#include "HciPump.h"
#include "Hci.h"

HciPump *HciPump::s_self = nullptr;

void HciPump::attach(Hci &hci) {
    if (m_attached) return;
    s_self = this; m_hci = &hci;
    m_responder.attach(serviceEvent);
    m_responder.triggerEvent();
    m_attached = true;
}

void HciPump::detach() {
    if (!m_attached) return;
    // clearEvent() BEFORE detach(): detach leaves _triggered set and a later
    // attach()+triggerEvent() would be a silent no-op (see WiFiClass::setAutoService).
    (void)m_responder.clearEvent();
    m_responder.detach();
    m_attached = false; m_hci = nullptr; s_self = nullptr;
}

void HciPump::serviceEvent(EventResponderRef ref) {
    if (s_self && s_self->m_hci) { s_self->m_hci->service(); s_self->m_passes++; }
    ref.triggerEvent();            // re-queue: one bounded pass per yield(), forever
}
