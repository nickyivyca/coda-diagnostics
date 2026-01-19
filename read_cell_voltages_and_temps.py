import time
import can
import argparse
import logging

BITRATE = 500000
REQ_ID  = 0x722
RESP_ID = 0x72A

logger = logging.getLogger("coda")
can_logger = logging.getLogger("can")


# -----------------------------------------------------------------------------
# Logging
# -----------------------------------------------------------------------------

def configure_logging(debug, debug_can):
    logging.basicConfig(format="%(message)s")
    logger.setLevel(logging.DEBUG if debug else logging.INFO)
    can_logger.setLevel(logging.INFO if debug_can else logging.WARNING)


# -----------------------------------------------------------------------------
# CAN interface detection (same as read_dtcs_coda.py)
# -----------------------------------------------------------------------------

def safe_shutdown(bus):
    try: bus.shutdown()
    except: pass
    try: bus.close()
    except: pass


def detect_can_interface():
    # IXXAT
    try:
        bus = can.interface.Bus(interface="ixxat", channel=0, bitrate=BITRATE)
        safe_shutdown(bus)
        return "ixxat", 0
    except:
        pass

    # PCAN
    for ch in ["PCAN_USBBUS1", "PCAN_USBBUS2"]:
        try:
            bus = can.interface.Bus(interface="pcan", channel=ch, bitrate=BITRATE)
            safe_shutdown(bus)
            return "pcan", ch
        except:
            pass

    # SocketCAN
    try:
        bus = can.interface.Bus(interface="socketcan", channel="can0", bitrate=BITRATE)
        safe_shutdown(bus)
        return "socketcan", "can0"
    except:
        pass

    # Kvaser
    try:
        bus = can.interface.Bus(interface="kvaser", channel=0, bitrate=BITRATE)
        safe_shutdown(bus)
        return "kvaser", 0
    except:
        pass

    raise RuntimeError("No CAN interface detected")


def create_bus(interface, channel):
    return can.interface.Bus(interface=interface, channel=channel, bitrate=BITRATE)


# -----------------------------------------------------------------------------
# ISO‑TP helpers (with streaming fix)
# -----------------------------------------------------------------------------

def send_isotp_sf(bus, req_id, payload):
    pci = 0x00 | len(payload)
    data = bytes([pci]) + payload
    data = data.ljust(8, b"\x00")
    logger.debug(f"CAN TX {req_id:03X} {data.hex()}")
    bus.send(can.Message(arbitration_id=req_id, data=data, is_extended_id=False))


def send_isotp_fc(bus, req_id):
    data = bytes([0x30, 0x00, 0x00]) + b"\x00"*5
    logger.debug(f"CAN TX {req_id:03X} {data.hex()} (FC)")
    bus.send(can.Message(arbitration_id=req_id, data=data, is_extended_id=False))


def recv_isotp(bus, resp_id, req_id, timeout=5.0):
    """
    Hardened ISO‑TP receiver:
    - Ignores stray consecutive frames until a First Frame arrives
    - Resyncs cleanly if the BMS is mid‑stream
    """
    buf = bytearray()
    expected_len = None
    sn = 1
    start = time.time()

    while True:
        if time.time() - start > timeout:
            return bytes(buf)

        msg = bus.recv(timeout=0.5)
        if msg is None or msg.arbitration_id != resp_id:
            continue

        d = bytes(msg.data)
        pci = d[0]
        ft = pci & 0xF0

        logger.debug(f"CAN RX {resp_id:03X} {d.hex()}")

        # Ignore stray consecutive frames until a First Frame is seen
        if expected_len is None and ft == 0x20:
            continue

        # Single frame
        if ft == 0x00:
            ln = pci & 0x0F
            p = d[1:1+ln]
            if len(p) >= 3 and p[0] == 0x7F and p[2] == 0x78:
                continue
            return p

        # First frame
        if ft == 0x10:
            expected_len = ((pci & 0x0F) << 8) | d[1]
            buf = bytearray(d[2:])
            sn = 1
            send_isotp_fc(bus, req_id)
            continue

        # Consecutive frame
        if ft == 0x20:
            if (pci & 0x0F) != (sn & 0x0F):
                # Resync: ignore and wait for next FF
                continue
            sn += 1
            buf.extend(d[1:])
            if expected_len and len(buf) >= expected_len:
                return bytes(buf[:expected_len])


# -----------------------------------------------------------------------------
# Parsers
# -----------------------------------------------------------------------------

def parse_cell_voltages(raw):
    # raw = 62 87 76 <data...>
    payload = raw[3:]
    cells = []
    for i in range(0, len(payload), 2):
        if i+1 >= len(payload):
            break
        v = (payload[i] << 8) | payload[i+1]
        cells.append(v / 1000.0)
    return cells


def parse_temperatures(raw):
    # raw = 62 87 78 <data...>
    payload = raw[3:]
    return list(payload)


# -----------------------------------------------------------------------------
# UDS wrappers
# -----------------------------------------------------------------------------

def uds_read(bus, did):
    send_isotp_sf(bus, REQ_ID, bytes([0x22, (did >> 8) & 0xFF, did & 0xFF]))
    return recv_isotp(bus, RESP_ID, REQ_ID)


# -----------------------------------------------------------------------------
# Column formatting
# -----------------------------------------------------------------------------

def print_in_columns(values, per_col, label):
    total = len(values)
    cols = (total + per_col - 1) // per_col

    print(f"\n=== {label} ===")

    for row in range(per_col):
        line = []
        for col in range(cols):
            idx = col * per_col + row
            if idx < total:
                v = values[idx]
                line.append(f"{idx+1:03d}: {v}")
            else:
                line.append("")
        print("   ".join(x.ljust(14) for x in line))


# -----------------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------------

def main():
    p = argparse.ArgumentParser(description="Read Coda BMS cell voltages + temperatures.")
    p.add_argument("--debug", action="store_true")
    p.add_argument("--debug-can-dongle", action="store_true")
    a = p.parse_args()

    configure_logging(a.debug, a.debug_can_dongle)

    iface, ch = detect_can_interface()
    print(f"Using interface={iface}, channel={ch}")
    bus = create_bus(iface, ch)

    try:
        print("\nRequesting cell voltages (DID 0x8776)...")
        raw_v = uds_read(bus, 0x8776)
        cells = parse_cell_voltages(raw_v)

        print("Requesting cell temperatures (DID 0x8778)...")
        raw_t = uds_read(bus, 0x8778)
        temps = parse_temperatures(raw_t)

        print_in_columns([f"{v:.3f} V" for v in cells], 26, "Cell Voltages (104s)")
        print_in_columns([f"{t} °C" for t in temps], 7, "Cell Temperatures (28 sensors)")

    finally:
        safe_shutdown(bus)


if __name__ == "__main__":
    main()
