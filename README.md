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

## Licence

MIT. Nothing is vendored here.
