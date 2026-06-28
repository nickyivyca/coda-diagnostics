# coda-diagnostics
Reverse engineering of the Coda Omitec dealer scan tool for use with USB to CAN dongles and Python scripts

Currently supports reading and clearing codes from the following:
- ABS
- AC Compressor (not very well tested since on my cars it has no codes)
- Airbag
- BMS
- DLCM (Drive Line Control Module, also contains codes for MCM [motor controller] and TCM [transmission park pin])
- Gateway (also contains DCDC)
- HVAC
- Power Steering

Does not support:
- Chargers (seems to be some sort of weird security handshake involved that the Omitec scanner uses)
- RCU (no clean DTC indication in the CAN messages logged, may implement when I can dig deeper)

Currently supported/tested CAN dongles:
- Kvaser (Windows)
- Ixxat (Windows)
- PCAN (Windows)
- gs_usb / candleLight, e.g. Innomaker USB2CAN (Windows)
- Socketcan (Linux)

Any CAN dongle supported by the Python CAN library should be able to be implemented however.

## Setup

Install dependencies with `pip install -r requirements.txt`.

The Innomaker USB2CAN (gs_usb firmware) needs **no vendor driver** on Windows —
it binds to WinUSB automatically via its WCID descriptors, so there is **no
Zadig step**. The `libusb-package` dependency bundles the `libusb-1.0.dll` that
pyusb needs, and `detect_can_interface()` puts it on the DLL search path at
runtime. It is probed after ixxat and before kvaser during auto-detection.
