// lwip netif over the M2Radio IW416 driver (W9).  The Wi-Fi sibling of the
// lwip repo's port/ethernetif.c: same NO_SYS=1 poll-loop model, same MAC
// convention (the sketch fills the C-linkage g_mac[6] before netif_add).
//
// Usage (mirrors lwip_test.cpp):
//   netif_add(&nif, IP4_ADDR_ANY4, IP4_ADDR_ANY4, IP4_ADDR_ANY4,
//             &iw416 /* state */, iw416NetifInit, ethernet_input);
//   ...
//   loop: if (!iw416NetifPoll(&nif)) { /* link dropped: reconnect, then
//            netif_set_link_up(&nif); */ } sys_check_timeouts();
#pragma once
#include "lwip/netif.h"
#include "lwip/err.h"

class Iw416;

// netif->state MUST be the Iw416* (netif_add's `state` argument).
err_t iw416NetifInit(struct netif *netif);

// One driver service pass: deliver every pending RX frame into lwip and
// record link events.  Returns false when the link dropped OR the SDIO bus
// errored (the netif is marked link-down; the caller owns reconnect +
// netif_set_link_up).
bool iw416NetifPoll(struct netif *netif);

// --- W17: a SECOND netif for the uAP (AP-mode) BSS ---------------------------
//
// The card runs both interfaces over ONE set of rings, tagged per packet in the
// RxPD (verified on silicon, W17: 99/99 uAP frames tagged bss_type=1 with none
// mis-tagged).  So the two netifs share a single service pass and are told
// apart by that tag, not by separate queues.
//
// ★ THE STA PATH ABOVE IS UNCHANGED, deliberately and checkably: iw416NetifInit
// and iw416NetifPoll are byte-for-byte what they were, so an existing STA-only
// sketch cannot notice this exists.  That is the W17 handoff's success
// criterion 3, and the cheapest way to meet it is not to touch the path.
//
// Usage, uAP side:
//   netif_add(&uapNif, &ip, &mask, &gw, &iw416, iw416NetifInitUap, ethernet_input);
//   netif_set_up(&uapNif); netif_set_link_up(&uapNif);
//   loop: iw416NetifPollDual(&staNif_or_NULL, &uapNif); sys_check_timeouts();
//
// ★ Do NOT also call iw416NetifPoll() when using the dual poll -- two service
// passes would race for the same ring and each would see half the frames.
err_t iw416NetifInitUap(struct netif *netif);

// One service pass for BOTH interfaces, routing each frame by its RxPD tag.
// Either netif may be NULL (uAP-only or STA-only).  A frame whose tag has no
// netif is counted and dropped rather than delivered to the wrong stack --
// mis-delivery is the specific hazard the W17 handoff flags about this path.
// Returns false on link-down/bus-error, exactly as iw416NetifPoll does.
bool iw416NetifPollDual(struct netif *sta, struct netif *uap);

// Frames dropped because their bss tag matched no registered netif.  Nonzero
// means the card is delivering traffic for an interface this sketch never
// created -- worth a print, and never worth silently ignoring.
unsigned long iw416NetifUnroutedFrames(void);
