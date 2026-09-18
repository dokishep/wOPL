# Vendored USB IRX (pre-USBD-rewrite)

These IRX modules are built from ps2sdk commit `2dc6b32f` (last tree
before `b1f7ff96` “USBD feature update”, 2024-09-04).

## Why

The PS2SDK September 2024 USBD rewrite broke multiple simultaneous USB devices
and causes hangs/black screens on USB mass storage and HID devices.
Stock OPL Beta-2125 and earlier used this FreeUsbd v0.1.2 driver.
Vendoring these stable IRXs restores normal multi-device USB enumeration and stability.

See:
- https://github.com/ps2homebrew/Open-PS2-Loader/issues/1751
- https://github.com/ps2homebrew/Open-PS2-Loader/pull/1752
