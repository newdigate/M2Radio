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

## `hci/` — the Bluetooth HCI transport (BT-1)

`import_evkb_library(M2Radio hci)` gives a sketch the host side of an HCI link
over `Serial2` (LPUART2 = the M.2 socket's BT UART): H4 framing, a command queue
that honours `Num_HCI_Command_Packets`, Command Complete/Status matching with
timeouts, and callbacks for asynchronous events and ACL data. `sdio`/`iw416`
are still needed to bring the card up — the Bluetooth firmware rides the combo
blob downloaded over SDIO — but `hci/` never compiles the Wi-Fi data path.

```cpp
static HciTransport io(Serial2);  static Hci hci(io);  static HciPump pump;
io.begin(115200); hci.begin(); pump.attach(hci);       // one service() per yield()
Hci::Reply r;
Hci::Error e = hci.run(0x0C03 /*Reset*/, nullptr, 0, &r, 500, [](){ delay(1); });
```

**Every exit is named** — `Hci::errorName()` gives `no_response`, `framing`,
`ncmd_starved`, `queue_full`, `status`, `busy` — and counted, because H4 has
no sync marker and LPUART2 has no flow control on this board: a lost byte
desyncs the stream for good, the parser's fault starts a 50 ms idle resync,
and the command in flight fails as `framing`, not `timeout`.

**An abandoned command's credit is given back, on a `timeout` as well as on a
`framing` fault.** `Num_HCI_Command_Packets` is assigned *absolutely* from
each reply, so a reply lost to a resync — or simply never sent — leaves
nothing able to raise the count back up: no credit means no command, and no
command means no reply. That is a permanent deadlock, not a slow recovery, and
it is what made a ten-attempt retry loop around `HCI_Reset` in the example
silently send only once, against a controller that never answers: attempt 1
spent the one startup credit and timed out without it, and attempts 2–10 never
dispatched at all — measured `timeouts=1 starved=9`, one command ever reaching
the wire. `H4Parser`, `Hci` and `HciEvents` are pure C++ with host unit tests
(`hci/test/run.sh`).

Example: `networking/m2_hci_probe` in the rt1176-evkb repo (card-absent gate,
a `[hci]` gate against `hci_peer.py`, and the silicon transcript).

## Licence

MIT. Nothing is vendored here.
