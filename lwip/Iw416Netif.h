// lwip netif over the M2Radio IW416 driver (W9).  The Wi-Fi sibling of the
// lwip repo's port/ethernetif.c: same NO_SYS=1 poll-loop model, same MAC
// convention (the sketch fills the C-linkage g_mac[6] before netif_add).
//
// Usage (mirrors lwip_test.cpp):
//   netif_add(&nif, IP4_ADDR_ANY4, IP4_ADDR_ANY4, IP4_ADDR_ANY4,
//             &iw416 /* state */, iw416NetifInit, ethernet_input);
//   ...
//   loop: if (!iw416NetifPoll(&nif)) { /* link dropped: reconnect, then
//            netif_set_link_up(&nif); } sys_check_timeouts();
#pragma once
#include "lwip/netif.h"
#include "lwip/err.h"

class Iw416;

// netif->state MUST be the Iw416* (netif_add's `state` argument).
err_t iw416NetifInit(struct netif *netif);

// One driver service pass: deliver every pending RX frame into lwip and
// record link events.  Returns false when the link dropped (the netif is
// marked link-down; the caller owns reconnect + netif_set_link_up).
bool iw416NetifPoll(struct netif *netif);
