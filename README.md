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

## Licence

MIT. Nothing is vendored here.
