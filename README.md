# M2Radio

SDIO host and NXP **IW416** support for the MIMXRT1170-EVKB M.2 socket (J54),
as used by the u-blox `M2-MAYA-W161-00C` card.

Subdirectories are imported selectively:

    import_evkb_library(M2Radio sdio)

* `sdio/` — a minimal SDIO host for uSDHC1: enumeration (CMD5/CMD3/CMD7),
  direct register access (CMD52), and CIS parsing. Independent of SdFat.

## ⚠️ uSDHC1 carries two card sockets

On the MIMXRT1170-EVKB the M.2 socket **and** the microSD slot J15 are wired in
parallel onto the same six MCU balls, every series resistor fitted. They cannot
both be used. **Remove any microSD card before using this library.**

There is no way to power the M.2 module down: `WL_3V3` comes through a ferrite
with no switch. Physical removal is the only isolation.

Full board map: `docs/m2-evkb-revc3.md` in the `rt1176-evkb` repo.

## Link servicing is POLLED by default

`Iw416::serviceLink()` reads `HOST_INT_STATUS` once per pass and paces itself on
`delay(1)` — roughly 1000 CMD52 per second on a completely idle link.
`Iw416::setInterruptMode(true)` replaces that with the SDIO card interrupt
(DAT1): the card says when it has work and quiet passes stop touching the bus
(measured in QEMU: ~9.6x fewer service CMD52 per received frame, idle rate
~1500/s to ~110/s).

**It defaults to off and should stay off until silicon says otherwise.** The
RT1176 uSDHC's card interrupt has never been exercised on a MIMXRT1170-EVKB, and
the QEMU model it was developed against derives DAT1 from the SDIO
specification rather than from a capture — so a green gate proves the driver
matches the spec as read, not that it matches the card. The polled path is
unchanged and is the same code either way, selected by one branch.

Either way the W12/W13 RD-bitmap safety net keeps running on the same schedule
(every 64 quiet passes): this firmware has been measured stranding uploads with
no interrupt, so nothing here trusts an interrupt completely.

## `arduino/` — the Arduino WiFi facade

`import_evkb_library(M2Radio sdio iw416 lwip arduino)` (the lwip library is
required too) gives a sketch the classic surface:

```cpp
WiFi.setFirmware(iw416_fw, iw416_fw_len);   // NXP blob, configure-time supplied
WiFi.begin(ssid, psk);                      // preamble + SDIO + fw + DHCP
WiFiClient c;  c.connect(ip, 80);  c.println("GET / HTTP/1.0");
WiFiServer s(80);  s.begin();
```

`WiFi.begin()` runs the **M.2 board preamble** (SDIO_RST / WL_RST-PDn release,
1.8 V pad switch) by default — the thing every pre-facade example had to
open-code, and whose omission is green in QEMU and dead on silicon.

### Four things it will not let you do casually, on purpose

* **Turn IEEE power save off.** PS-on is the W10 workaround for the firmware
  idle-RX-death erratum. There is no switch; `WiFi.radio().setIeeePs()` puts
  you next to the erratum comment.
* **Starve the link.** A `yield()`-driven pump services the stack from every
  `loop()` pass and every `delay()` millisecond. `WiFi.setAutoService(false)` +
  `WiFi.loop()` if you want the cadence. **Expect `loop()` to run at ~1 kHz
  once the link is up** — a quiet service pass blocks ~1 ms in the driver.
* **Poll without consuming.** `available()`/`read()`/`peek()`/`connected()`
  short-circuit when bytes are staged (that is what makes byte-at-a-time
  reading fast — 31 service passes down to 1 over a 15-byte drain), so a loop
  that polls and never consumes stops servicing the link *entirely*. It
  presents as a dead radio, not a stalled read.
* **Mix with the Ethernet library.** `arduino/` carries its own clean-room MIT
  `Client`/`Server`/`IPAddress`, as NativeEthernet does, so importing both
  collides on `class Client`.

### Two semantics that differ from upstream Ethernet/WiFiNINA

* **A connection lives as long as its handle.** `WiFiClient` is refcounted and
  the pool closes the connection when the last handle dies, so a transient
  `WiFiClient c = server.available();` inside `loop()` is a **one-shot**
  server. That is the canonical Arduino request/response shape, where the
  handle's scope *is* the session — it is only wrong if you expected a second
  message on the same socket. Hold the handle in a `static` to keep it.
* **`available()` and `accept()` are not interchangeable.** `available()` only
  surfaces a connection with staged bytes, so silent ones stay unclaimed and
  therefore reapable by the pool's valves. `accept()` surfaces silent ones too,
  and claiming exempts them from both valves — an `accept()`-based server can
  claim all four slots in a millisecond and then refuse everything.

### Diagnosis

The layer is built to be read from a serial transcript: `WiFi.status()`,
`WiFiClient::lastError()` (7 causes), `WiFiServer::lastError()` (5 failures),
and three pool counters — `WiFiPool::evictions()`, `stallAborts()` and
`acceptRefusals()`, one per way a connection can vanish without the sketch
asking. `WiFi.radio()` and `WiFi.sdio()` stay exposed so the facade is a floor,
not a ceiling.

Examples: `networking/wifi_client_test` (client echo + a held `accept()`
session) and `networking/wifi_server_test` (one-shot `available()` server, with
a Mac-side `wifi_peer.py` as the authoritative peer).

## Licence

MIT. Nothing is vendored here.
