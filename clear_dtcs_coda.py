import time
import can
import argparse
import logging

BITRATE = 500000

MODULES = {
    "BMS":          {"req": 0x722, "resp": 0x72A, "type": "uds14"},
    "DLCM":         {"req": 0x7E0, "resp": 0x7E8, "type": "uds14"},
    "Gateway":      {"req": 0x710, "resp": 0x718, "type": "uds14"},
    "Airbag":       {"req": 0x7C4, "resp": 0x7CC, "type": "uds14_ignition_cycle"},
    "HVAC":         {"req": 0x750, "resp": 0x758, "type": "uds14"},
    "DCDC":         {"req": 0x710, "resp": 0x718, "type": "uds14"},
    "ACCompressor": {"req": 0x7C7, "resp": 0x7CF, "type": "uds14"},
    "ABS":          {"req": 0x784, "resp": 0x785, "type": "uds14_ignition_cycle"},
    "PowerSteering":{"req": 0x724, "resp": 0x734, "type": "eps_special"},
}

FLAG_MAP = {
    "BMS": "bms",
    "DLCM": "dlcm",
    "ABS": "abs",
    "Airbag": "airbag",
    "HVAC": "hvac",
    "Gateway": "gateway",
    "DCDC": "dcdc",
    "PowerSteering": "eps",
    "ACCompressor": "accompressor",
}

logger = logging.getLogger("coda")
can_logger = logging.getLogger("can")

def configure_logging(debug, debug_can):
    logging.basicConfig(format="%(message)s")
    logger.setLevel(logging.DEBUG if debug else logging.INFO)
    can_logger.setLevel(logging.INFO if debug_can else logging.WARNING)

def safe_shutdown(bus):
    try: bus.shutdown()
    except: pass
    try: bus.close()
    except: pass

def detect_can_interface():
    try:
        b = can.interface.Bus(interface="ixxat", channel=0, bitrate=BITRATE)
        safe_shutdown(b)
        return "ixxat", 0
    except: pass
    for ch in ["PCAN_USBBUS1", "PCAN_USBBUS2"]:
        try:
            b = can.interface.Bus(interface="pcan", channel=ch, bitrate=BITRATE)
            safe_shutdown(b)
            return "pcan", ch
        except: pass
    try:
        b = can.interface.Bus(interface="socketcan", channel="can0", bitrate=BITRATE)
        safe_shutdown(b)
        return "socketcan", "can0"
    except: pass
    try:
        b = can.interface.Bus(interface="kvaser", channel=0, bitrate=BITRATE)
        safe_shutdown(b)
        return "kvaser", 0
    except: pass
    raise RuntimeError("No CAN interface detected")

def create_bus(interface, channel):
    return can.interface.Bus(interface=interface, channel=channel, bitrate=BITRATE)

def send_isotp_sf(bus, req_id, payload):
    pci = 0x00 | len(payload)
    data = bytes([pci]) + payload
    data = data.ljust(8, b"\x00")
    logger.debug(f"TX {req_id:03X} {data.hex()}")
    bus.send(can.Message(arbitration_id=req_id, data=data, is_extended_id=False))

def send_isotp_fc(bus, req_id):
    data = bytes([0x30, 0x00, 0x00]) + b"\x00"*5
    logger.debug(f"TX {req_id:03X} {data.hex()} (FC)")
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
        logger.debug(f"RX {msg.arbitration_id:03X} {msg.data.hex()}")
        d = bytes(msg.data)
        pci = d[0]
        ft = pci & 0xF0
        if len(d) >= 3 and d[0] == 0x7F and d[2] == 0x78:
            continue
        if ft == 0x00:
            ln = pci & 0x0F
            return d[1:1+ln]
        if ft == 0x10:
            exp = ((pci & 0x0F)<<8) | d[1]
            buf = d[2:]
            send_isotp_fc(bus, req_id)
            continue
        if ft == 0x20:
            if (pci & 0x0F) != (sn & 0x0F):
                raise ValueError("Bad SN")
            sn += 1
            buf += d[1:]
            if exp and len(buf) >= exp:
                return buf[:exp]

def uds14_clear(bus, req, resp):
    send_isotp_sf(bus, req, bytes([0x14, 0xFF, 0xFF, 0xFF]))
    return recv_isotp(bus, resp, req)

def uds19_read(bus, req, resp):
    send_isotp_sf(bus, req, bytes([0x19, 0x02, 0x0C]))
    return recv_isotp(bus, resp, req)

def eps_clear_sequence(bus, req, resp):
    send_isotp_sf(bus, req, bytes([0x31, 0x01, 0x03]))
    recv_isotp(bus, resp, req)
    send_isotp_sf(bus, req, bytes([0x31, 0x02, 0x03]))
    recv_isotp(bus, resp, req)
    return True

def gateway_read_ignition(bus):
    send_isotp_sf(bus, 0x710, bytes([0x22, 0x64, 0x01]))
    r = recv_isotp(bus, 0x718, 0x710)
    if len(r) >= 4:
        return r[3]
    return None

def wait_for_ignition_off(bus):
    print("  Turn ignition OFF...")
    start = time.time()
    while time.time() - start < 30:
        st = gateway_read_ignition(bus)
        if st == 0x00:
            print("  Ignition OFF detected.")
            return True
        time.sleep(0.5)
    return False

def wait_for_ignition_on(bus):
    print("  Turn ignition ON...")
    start = time.time()
    while time.time() - start < 30:
        st = gateway_read_ignition(bus)
        if st == 0x03:
            print("  Ignition ON detected.")
            return True
        time.sleep(0.5)
    return False

def dlcm_no_dtcs(dtc_bytes):
    if not dtc_bytes:
        return True
    if dtc_bytes[:3] == b"\x59\x02\xDF" and all(b == 0x55 for b in dtc_bytes[3:]):
        return True
    return False
def clear_module(bus, name, ids):
    print(f"\n=== Clearing {name} ===")
    try:
        if ids["type"] == "uds14":
            r = uds14_clear(bus, ids["req"], ids["resp"])
            if not r:
                print(f"  {name}: No response to clear command")
                return "fail"

            time.sleep(0.2)
            dtc = uds19_read(bus, ids["req"], ids["resp"])

            if name == "DLCM":
                if dlcm_no_dtcs(dtc):
                    print(f"  {name}: Cleared")
                    return "ok"
                else:
                    print(f"  {name}: DTCs remain (module did not clear all codes)")
                    return "ok_remaining"

            if dtc and dtc != b"":
                print(f"  {name}: DTCs remain (module did not clear all codes)")
                return "ok_remaining"

            print(f"  {name}: Cleared")
            return "ok"

        if ids["type"] == "uds14_ignition_cycle":
            r = uds14_clear(bus, ids["req"], ids["resp"])
            if not r:
                print(f"  {name}: No response to clear command")
                return "fail"

            print(f"  {name}: Clear accepted. Ignition cycle required.")

            if not wait_for_ignition_off(bus):
                print(f"  {name}: Ignition OFF not detected")
                return "fail"

            if not wait_for_ignition_on(bus):
                print(f"  {name}: Ignition ON not detected")
                return "fail"

            dtc = uds19_read(bus, ids["req"], ids["resp"])
            if dtc and dtc != b"":
                print(f"  {name}: DTCs remain (module did not clear all codes)")
                return "ok_remaining"

            print(f"  {name}: Cleared")
            return "ok"

        if ids["type"] == "eps_special":
            eps_clear_sequence(bus, ids["req"], ids["resp"])
            print("  PowerSteering: Clear accepted. Ignition cycle required.")

            if not wait_for_ignition_off(bus):
                print("  PowerSteering: Ignition OFF not detected")
                return "fail"

            if not wait_for_ignition_on(bus):
                print("  PowerSteering: Ignition ON not detected")
                return "fail"

            print("  PowerSteering: Ignition cycle complete")
            return "ok"

        print(f"  {name}: Unsupported clear type")
        return "fail"

    except Exception as e:
        print(f"  {name}: Error {e}")
        return "fail"

def main():
    p = argparse.ArgumentParser(
        description="Clear DTCs on Coda modules.",
        formatter_class=argparse.RawTextHelpFormatter,
        allow_abbrev=False
    )

    p.add_argument("--bms", help="Clear BMS DTCs", action="store_true")
    p.add_argument("--dlcm", help="Clear DLCM DTCs", action="store_true")
    p.add_argument("--abs", help="Clear ABS DTCs (requires ignition cycle)", action="store_true")
    p.add_argument("--airbag", help="Clear Airbag DTCs", action="store_true")
    p.add_argument("--hvac", help="Clear HVAC DTCs", action="store_true")
    p.add_argument("--gateway", help="Clear Gateway DTCs", action="store_true")
    p.add_argument("--dcdc", help="Clear DCDC DTCs", action="store_true")
    p.add_argument("--eps", help="Clear EPS DTCs (requires ignition cycle)", action="store_true")
    p.add_argument("--accompressor", help="Clear AC Compressor DTCs", action="store_true")
    p.add_argument("--all", help="Clear all modules", action="store_true")

    p.add_argument("--debug", help="Enable script debug logging", action="store_true")
    p.add_argument("--debug-can-dongle", help="Enable python-can dongle logging", action="store_true")

    a = p.parse_args()

    configure_logging(a.debug, a.debug_can_dongle)

    try:
        iface, ch = detect_can_interface()
    except Exception as e:
        print(f"Error detecting CAN interface: {e}")
        return

    bus = create_bus(iface, ch)
    print(f"Using interface={iface}, channel={ch}")

    selected = []

    if a.all:
        selected = list(MODULES.keys())
    else:
        for m in MODULES:
            flag = FLAG_MAP[m]
            if getattr(a, flag):
                selected.append(m)

    if not selected:
        print("No modules selected.")
        safe_shutdown(bus)
        return

    results = {}

    try:
        for m in selected:
            status = clear_module(bus, m, MODULES[m])
            results[m] = status
    finally:
        safe_shutdown(bus)

    print("\nSummary:")
    for m in MODULES:
        if m in results:
            status = results[m]
            if status == "ok":
                print(f"  {m}: OK")
            elif status == "ok_remaining":
                print(f"  {m}: OK (DTCs remain)")
            else:
                print(f"  {m}: FAIL")

if __name__ == "__main__":
    main()
