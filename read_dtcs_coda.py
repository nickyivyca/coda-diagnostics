import time
import can
import argparse
import logging
import csv
from collections import defaultdict

BITRATE = 500000
CSV_PATH = "coda_dtc_list.csv"

MODULES = {
    "ABS": {
        "req": 0x784,
        "resp": 0x785,
        "type": "uds18",
        "sub": 0x00,
        "mask": 0xFF,
    },
    "ACCompressor": {
        "req": 0x7C7,
        "resp": 0x7CF,
        "type": "uds19",
    },
    "Airbag": {
        "req": 0x7C4,
        "resp": 0x7CC,
        "type": "uds18",
        "sub": 0x02,
        "mask": 0xFF,
    },
    "BMS": {
        "req": 0x722,
        "resp": 0x72A,
        "type": "uds19",
    },
    "DLCM": {
        "req": 0x7E0,
        "resp": 0x7E8,
        "type": "uds19",
    },
    "Gateway": {
        "req": 0x710,
        "resp": 0x718,
        "type": "uds19",
    },
    "HVAC": {
        "req": 0x750,
        "resp": 0x758,
        "type": "uds19",
    },
    "PowerSteering": {
        "req": 0x724,
        "resp": 0x734,
        "type": "uds19",
    },
}

logger = logging.getLogger("coda")
can_logger = logging.getLogger("can")


def configure_logging(debug, debug_can):
    logging.basicConfig(format="%(message)s")
    logger.setLevel(logging.DEBUG if debug else logging.INFO)
    can_logger.setLevel(logging.INFO if debug_can else logging.WARNING)


def safe_shutdown(bus):
    try:
        bus.shutdown()
    except:
        pass
    try:
        bus.close()
    except:
        pass


def load_dtc_csv():
    db = defaultdict(dict)
    try:
        with open(CSV_PATH, newline="") as f:
            r = csv.DictReader(f)
            for row in r:
                m = row["module"].strip()
                d = row["dtc"].strip().upper()
                desc = row["description"].strip()
                if not m or not d:
                    continue
                if d in db[m]:
                    if desc and desc not in db[m][d]:
                        db[m][d] += " OR " + desc
                else:
                    db[m][d] = desc
    except FileNotFoundError:
        print(f"Warning: {CSV_PATH} not found.")
    return db


def detect_can_interface():
    try:
        bus = can.interface.Bus(interface="ixxat", channel=0, bitrate=BITRATE)
        safe_shutdown(bus)
        return "ixxat", 0
    except:
        pass
    for ch in ["PCAN_USBBUS1", "PCAN_USBBUS2"]:
        try:
            bus = can.interface.Bus(interface="pcan", channel=ch, bitrate=BITRATE)
            safe_shutdown(bus)
            return "pcan", ch
        except:
            pass
    try:
        bus = can.interface.Bus(interface="socketcan", channel="can0", bitrate=BITRATE)
        safe_shutdown(bus)
        return "socketcan", "can0"
    except:
        pass
    try:
        bus = can.interface.Bus(interface="kvaser", channel=0, bitrate=BITRATE)
        safe_shutdown(bus)
        return "kvaser", 0
    except:
        pass
    raise RuntimeError("No CAN interface detected")


def create_bus(interface, channel):
    return can.interface.Bus(interface=interface, channel=channel, bitrate=BITRATE)


def send_isotp_sf(bus, req_id, payload):
    pci = 0x00 | len(payload)
    data = bytes([pci]) + payload
    data = data.ljust(8, b"\x00")
    logger.debug(f"CAN TX {req_id:03X} {data.hex()}")
    bus.send(can.Message(arbitration_id=req_id, data=data, is_extended_id=False))


def send_isotp_fc(bus, req_id):
    data = bytes([0x30, 0x00, 0x00]) + b"\x00" * 5
    logger.debug(f"CAN TX {req_id:03X} {data.hex()} (FC)")
    bus.send(can.Message(arbitration_id=req_id, data=data, is_extended_id=False))


def recv_isotp(bus, resp_id, req_id, timeout=5.0):
    buf = b""
    exp = None
    sn = 1
    start = time.time()
    while True:
        if time.time() - start > timeout:
            return buf
        msg = bus.recv(timeout=0.5)
        if msg is None or msg.arbitration_id != resp_id:
            continue
        logger.debug(f"CAN RX {msg.arbitration_id:03X} {msg.data.hex()}")
        d = bytes(msg.data)
        pci = d[0]
        ft = pci & 0xF0

        if ft == 0x00:
            # Single frame: payload starts at d[1]
            ln = pci & 0x0F
            p = d[1:1 + ln]

            # Filter UDS response pending: 7F xx 78
            if len(p) >= 3 and p[0] == 0x7F and p[2] == 0x78:
                continue

            return p

        if ft == 0x10:
            # First frame
            exp = ((pci & 0x0F) << 8) | d[1]
            buf = d[2:]
            send_isotp_fc(bus, req_id)
            continue

        if ft == 0x20:
            # Consecutive frame
            if (pci & 0x0F) != (sn & 0x0F):
                raise ValueError("Bad SN")
            sn += 1
            buf += d[1:]
            if exp and len(buf) >= exp:
                return buf[:exp]



def uds19(bus, req, resp):
    send_isotp_sf(bus, req, bytes([0x19, 0x02, 0x0C]))
    return recv_isotp(bus, resp, req)


def uds18(bus, req, resp, subfunc, mask):
    payload = bytes([0x18, subfunc, mask, 0x00])
    send_isotp_sf(bus, req, payload)
    return recv_isotp(bus, resp, req)


def parse_bms(raw):
    if raw[0] != 0x59 or raw[1] != 0x02:
        raise ValueError("Bad BMS")
    mask = raw[2]
    out = []
    i = 3
    while i + 4 <= len(raw):
        dtc = (raw[i] << 16) | (raw[i+1] << 8) | raw[i+2]
        st = raw[i+3]
        out.append((dtc, st))
        i += 4
    return mask, out


def parse_dlcm(raw):
    if raw[0] != 0x59 or raw[1] != 0x02:
        raise ValueError("Bad DLCM")
    out = []
    i = 2
    while i + 4 <= len(raw):
        dtc = (raw[i] << 16) | (raw[i+1] << 8) | raw[i+2]
        st = raw[i+3]
        out.append((dtc, st))
        i += 4
    return 0, out


def parse_eps(raw):
    if raw[0] != 0x59 or raw[1] != 0x02:
        raise ValueError("Bad EPS")
    out = []
    i = 2
    while i + 3 <= len(raw):
        dtc = (raw[i] << 8) | raw[i+1]
        st = raw[i+2]
        out.append((dtc, st))
        i += 3
    return 0, out


def parse_abs(raw):
    """
    Parse ABS UDS-18 DTC response.
    Expected payload (after ISO-TP reassembly):
        58 <status_mask> <DTC_H> <DTC_L> <status> ...
    Continental ABS encodes status bits in the high byte (0x40, 0x80).
    These must be stripped to recover the true DTC family.
    """

    # Must begin with positive response 0x58
    if not raw or raw[0] != 0x58:
        raise ValueError("Bad ABS")

    status_mask = raw[1]
    dtc_bytes = raw[2:]

    out = []
    i = 0

    while i + 1 < len(dtc_bytes):
        # Extract raw DTC bytes
        hi = dtc_bytes[i]
        lo = dtc_bytes[i+1]

        # Strip ABS status bits (0x40, 0x80)
        hi &= 0x3F

        dtc = (hi << 8) | lo

        # Status byte is optional; default to 0x00 if missing
        st = dtc_bytes[i+2] if i + 2 < len(dtc_bytes) else 0x00

        out.append((dtc, st))
        i += 3

    return status_mask, out



def parse_airbag(raw):
    # Same UDS18 response format as ABS: 58 <mask> <DTC_H> <DTC_L> <status> ...
    if not raw or raw[0] != 0x58:
        raise ValueError("Bad Airbag")
    status_mask = raw[1]
    dtc_bytes = raw[2:]
    out = []
    i = 0
    while i + 2 < len(dtc_bytes):
        dtc = (dtc_bytes[i] << 8) | dtc_bytes[i+1]
        st = dtc_bytes[i+2]
        out.append((dtc, st))
        i += 3
    return status_mask, out


def dec_bms(d):
    b0 = (d >> 16) & 0xFF
    b1 = (d >> 8) & 0xFF
    return f"P{(b0>>4):X}{(b0&0xF):X}{(b1>>4):X}{(b1&0xF):X}"


def dec_dlcm(d):
    x = d & 0xFFFF
    x2 = x & ~0x4000
    s = (x2 >> 12) & 0xF
    d2 = (x2 >> 8) & 0xF
    d3 = (x2 >> 4) & 0xF
    d4 = x2 & 0xF
    if s == 0x8:
        return f"U0{d2:X}{d3:X}{d4:X}"
    if s == 0x0:
        return f"P0{d2:X}{d3:X}{d4:X}"
    if s == 0x1:
        return f"C0{d2:X}{d3:X}{d4:X}"
    if s == 0x2:
        return f"C2{d2:X}{d3:X}{d4:X}"
    if s == 0x3:
        return f"P3{d2:X}{d3:X}{d4:X}"
    return f"?{s:X}{d2:X}{d3:X}{d4:X}"


def dec_chassis16(d):
    # Generic 16‑bit chassis style mapping (used by Gateway, HVAC, Airbag)
    x = d & 0xFFFF
    s = (x >> 12) & 0xF
    sys = "B" if s in (8, 9, 0xA, 0xB) else "?"
    pref = "1" if s in (8, 9) else ("2" if s in (0xA, 0xB) else "1")
    d1 = (x >> 8) & 0xF
    d2 = (x >> 4) & 0xF
    d3 = x & 0xF
    return f"{sys}{pref}{d1:X}{d2:X}{d3:X}"


def dec_eps(d):
    s = (d >> 12) & 0xF
    d1 = (d >> 8) & 0xF
    d2 = (d >> 4) & 0xF
    d3 = d & 0xF
    if s == 0x8:
        return "C1300"
    if s == 0xC:
        return f"C1{d1:X}{d2:X}{d3:X}"
    return f"C{d:04X}"


def dec_abs(d):
    hi = (d >> 8) & 0xFF
    lo = d & 0xFF

    if hi == 0x50:
        return f"C10{lo:02X}"

    if hi == 0x02:
        sw = ((lo & 0x0F) << 4) | ((lo & 0xF0) >> 4)
        return f"C22{sw:02X}"

    # 0x62xx family → C22xx (ABS quirk)
    if hi == 0x62:
        return f"C22{lo:02X}"

    if hi == 0x70:
        return f"U00{lo:02X}"
    if hi == 0x71:
        return f"U10{lo:02X}"
    if hi == 0x72:
        return f"U20{lo:02X}"

    return f"C{d:04X}"


def main():
    p = argparse.ArgumentParser(description="Read DTCs from Coda modules.")
    p.add_argument("--debug", action="store_true")
    p.add_argument("--debug-can-dongle", action="store_true")
    p.add_argument("--show-statuses", action="store_true")
    a = p.parse_args()

    configure_logging(a.debug, a.debug_can_dongle)
    db = load_dtc_csv()

    try:
        iface, ch = detect_can_interface()
    except Exception as e:
        print(f"Error detecting CAN interface: {e}")
        return

    bus = create_bus(iface, ch)
    print(f"Using interface={iface}, channel={ch}")

    try:
        for name, ids in MODULES.items():
            print(f"\n=== Reading DTCs from {name} ===")
            try:
                if ids["type"] == "uds19":
                    raw = uds19(bus, ids["req"], ids["resp"])
                else:
                    raw = uds18(bus, ids["req"], ids["resp"], ids["sub"], ids["mask"])

                if not raw:
                    print(f"  {name}: No response")
                    continue

                # Parse
                if name == "BMS":
                    mask, dtcs = parse_bms(raw)
                elif name == "ABS":
                    mask, dtcs = parse_abs(raw)
                elif name == "Airbag":
                    mask, dtcs = parse_airbag(raw)
                elif name == "PowerSteering":
                    mask, dtcs = parse_eps(raw)
                else:
                    mask, dtcs = parse_dlcm(raw)

                if not dtcs:
                    print(f"  {name}: No DTCs")
                    continue

                # Decode + print
                for d, st in dtcs:
                    if name == "BMS":
                        code = dec_bms(d)
                    elif name == "Airbag":
                        code = dec_chassis16(d)
                    elif name == "ABS":
                        code = dec_abs(d)
                    elif name in ("Gateway", "HVAC"):
                        code = dec_chassis16(d)
                    elif name == "PowerSteering":
                        code = dec_eps(d)
                    else:
                        code = dec_dlcm(d)

                    desc = db.get(name, {}).get(code, "Unknown DTC")

                    if a.show_statuses:
                        print(f"  {code} – {desc} (status=0x{st:02X})")
                    else:
                        print(f"  {code} – {desc}")

            except Exception as e:
                print(f"Error reading {name}: {e}")

    finally:
        safe_shutdown(bus)


if __name__ == "__main__":
    main()
