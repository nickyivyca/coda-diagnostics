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
