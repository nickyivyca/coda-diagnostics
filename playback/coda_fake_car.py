#!/usr/bin/env python3
import time
import can
import argparse
import re

BITRATE = 500000

# =============================================================================
#  DEBUG HELPERS
# =============================================================================

def dbg_tx(arb, data):
    print(f"CAN TX {arb:03X} {data.hex()}")

def dbg_rx(arb, data):
    print(f"CAN RX {arb:03X} {data.hex()}")


# =============================================================================
#  RAW LOG LOADERS
# =============================================================================

def load_abs_raw_busmaster(path):
    """
    Load raw ABS frames from a BusMaster log.
    Only keeps frames with ID 0x785.
    """
    frames = []
    with open(path, "r") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("***"):
                continue

            parts = line.split()
            if len(parts) < 7:
                continue

            try:
                can_id_str = parts[3]
                if not can_id_str.lower().startswith("0x"):
                    continue
                arb_id = int(can_id_str, 16)
            except Exception:
                continue

            if arb_id != 0x785:
                continue

            try:
                dlc = int(parts[5])
                data_bytes = parts[6:6+dlc]
                data = bytes(int(b, 16) for b in data_bytes)
            except Exception:
                continue

            frames.append((arb_id, data))

    print(f"[ABS-RAW-BUSMASTER] Loaded {len(frames)} frames from {path}")
    return frames


def _load_raw_txt(path, expected_id):
    """
    Generic raw frame loader for our own script logs.
    Example line:
      CAN RX 785 1008580261002062
    Keeps only frames with given expected_id.
    """
    frames = []
    pattern = re.compile(r"CAN\s+(RX|TX)\s+([0-9A-Fa-f]+)\s+([0-9A-Fa-f]+)")
    with open(path, "r") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue

            m = pattern.match(line)
            if not m:
                continue

            _dir, id_str, data_hex = m.groups()
            try:
                arb_id = int(id_str, 16)
            except Exception:
                continue

            if arb_id != expected_id:
                continue

            try:
                data = bytes.fromhex(data_hex)
            except Exception:
                continue

            frames.append((arb_id, data))

    return frames


def load_abs_raw_txt(path):
    frames = _load_raw_txt(path, 0x785)
    print(f"[ABS-RAW-TXT] Loaded {len(frames)} frames from {path}")
    return frames


def load_ac_raw_txt(path):
    frames = _load_raw_txt(path, 0x7CF)
    print(f"[AC-RAW-TXT] Loaded {len(frames)} frames from {path}")
    return frames


def load_airbag_raw_busmaster(path):
    """
    Load raw Airbag frames from a BusMaster log.
    Only keeps frames with ID 0x7CC (airbag response).
    """
    frames = []
    with open(path, "r") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("***"):
                continue

            parts = line.split()
            if len(parts) < 7:
                continue

            try:
                can_id_str = parts[3]
                if not can_id_str.lower().startswith("0x"):
                    continue
                arb_id = int(can_id_str, 16)
            except Exception:
                continue

            if arb_id != 0x7CC:
                continue

            try:
                dlc = int(parts[5])
                data_bytes = parts[6:6+dlc]
                data = bytes(int(b, 16) for b in data_bytes)
            except Exception:
                continue

            frames.append((arb_id, data))

    print(f"[AIRBAG-RAW-BUSMASTER] Loaded {len(frames)} frames from {path}")
    return frames


def load_airbag_raw_txt(path):
    frames = _load_raw_txt(path, 0x7CC)
    print(f"[AIRBAG-RAW-TXT] Loaded {len(frames)} frames from {path}")
    return frames


# =============================================================================
#  COMMON SAE DTC PARSER
# =============================================================================

def parse_sae_dtc(s):
    """
    ABS/Airbag-specific DTC parsing:
    Take the last 4 characters as a pure hex 16-bit value.
    Example:
      C102B -> 0x102B
      C2100 -> 0x2100
      U0155 -> 0x0155
    """
    s = s.strip().upper()
    if len(s) != 5:
        raise ValueError(f"Invalid DTC format: {s}")

    digits = s[1:]
    raw = int(digits, 16)
    return raw


# =============================================================================
#  ABS SYNTHETIC DTC ENCODING
# =============================================================================

def encode_abs_dtc_from_sae(sae_code, multi=False):
    """
    Continental ABS synthetic encoder.
    Single-DTC:
      - mask = 0x01
      - status = 0xE0
    Multi-DTC:
      - mask = 0x02
      - status = 0x20
    """
    raw = parse_sae_dtc(sae_code)

    hi = (raw >> 8) & 0xFF
    lo = raw & 0xFF

    hi |= 0x40

    if multi:
        status = 0x20
    else:
        status = 0xE0

    return bytes([hi, lo, status])


def build_abs_synthetic_response(dtc_list):
    """
    Build UDS-18 payload for ABS synthetic DTCs.
    Returns list of (arb_id, data) frames (0x785).
    """
    payload = bytearray()
    payload.append(0x58)  # Positive response to 0x18

    multi = len(dtc_list) > 1
    if multi:
        payload.append(0x02)  # multi-DTC mask (matches real ABS multi)
    else:
        payload.append(0x01)  # single-DTC mask (matches real single)

    for code in dtc_list:
        payload.extend(encode_abs_dtc_from_sae(code, multi=multi))

    frames = []

    # Single-frame response
    if len(payload) <= 7:
        sf_pci = 0x00 | len(payload)
        data = bytes([sf_pci]) + payload
        data = data.ljust(8, b"\x00")
        frames.append((0x785, data))
        return frames

    # Multi-frame response
    total_len = len(payload)
    ff = bytes([0x10 | (total_len >> 8), total_len & 0xFF]) + payload[:6]
    ff = ff.ljust(8, b"\x00")
    frames.append((0x785, ff))

    idx = 6
    sn = 1
    while idx < len(payload):
        chunk = payload[idx:idx+7]
        cf = bytes([0x20 | (sn & 0x0F)]) + chunk
        cf = cf.ljust(8, b"\x00")
        frames.append((0x785, cf))
        idx += 7
        sn += 1

    return frames


# =============================================================================
#  AIRBAG SYNTHETIC DTC ENCODING
# =============================================================================

def encode_airbag_dtc_from_sae(sae_code, multi=False):
    """
    For now use same DTC triplet encoding rules as ABS:
      hi/lo from last 4 digits, hi |= 0x40, status depends on multi.
    """
    raw = parse_sae_dtc(sae_code)

    hi = (raw >> 8) & 0xFF
    lo = raw & 0xFF

    hi |= 0x40

    if multi:
        status = 0x20
    else:
        status = 0xE0

    return bytes([hi, lo, status])


def build_airbag_synthetic_response(dtc_list):
    """
    Build UDS-18 payload for Airbag synthetic DTCs on 0x7CC.
    Airbag behavior from logs:
      - Request: 7C4 04 18 02 FF ...
      - First response: 7CC 03 7F 18 78 00 00 00 00 (response pending)
      - Second response: 7CC 05 58 01 A5 05 E0 00 00 (final)
    We will mimic this by:
      - Always sending a 7F 18 78 pending frame first.
      - Then sending a 58 final response, with 0 or more triplets.
    """
    frames = []

    # 7F 18 78 (response pending) single-frame
    pending_payload = bytes([0x7F, 0x18, 0x78])
    pending_pci = 0x00 | len(pending_payload)
    pending = bytes([pending_pci]) + pending_payload
    pending = pending.ljust(8, b"\x00")
    frames.append((0x7CC, pending))

    # Final 58 response
    payload = bytearray()
    payload.append(0x58)  # Positive response to 0x18

    if not dtc_list:
        # Match your "no DTCs" case: mask=0x00, no triplets
        payload.append(0x00)
    else:
        multi = len(dtc_list) > 1
        if multi:
            payload.append(0x02)
        else:
            payload.append(0x01)

        for code in dtc_list:
            payload.extend(encode_airbag_dtc_from_sae(code, multi=multi))

    if len(payload) <= 7:
        sf_pci = 0x00 | len(payload)
        data = bytes([sf_pci]) + payload
        data = data.ljust(8, b"\x00")
        frames.append((0x7CC, data))
        return frames

    # Multi-frame airbag not observed yet, but we can add later if needed.
    total_len = len(payload)
    ff = bytes([0x10 | (total_len >> 8), total_len & 0xFF]) + payload[:6]
    ff = ff.ljust(8, b"\x00")
    frames.append((0x7CC, ff))

    idx = 6
    sn = 1
    while idx < len(payload):
        chunk = payload[idx:idx+7]
        cf = bytes([0x20 | (sn & 0x0F)]) + chunk
        cf = cf.ljust(8, b"\x00")
        frames.append((0x7CC, cf))
        idx += 7
        sn += 1

    return frames


# =============================================================================
#  ISO-TP MULTIFRAME SENDER (ABS, OPTION B)
# =============================================================================

def send_isotp_multiframe(bus, frames):
    """
    frames = list of (arb_id, data)
    frames[0] must be FF, remaining are CFs.

    Option B behavior:
      - Send FF
      - Pause ~30 ms (like real ABS)
      - Wait up to 1s for FC (ID 0x784, PCI 0x30)
        - If FC arrives: honor STmin
        - If no FC: still send CFs with small default gap
    """
    ff_arb, ff_data = frames[0]

    # Send First Frame
    dbg_tx(ff_arb, ff_data)
    bus.send(can.Message(arbitration_id=ff_arb, data=ff_data, is_extended_id=False))

    # Give the scanner time to send FC (real ABS pauses here)
    time.sleep(0.03)

    # Default behavior if no FC
    st_min = 0.001  # 1 ms fallback

    # Try to receive FC
    fc = bus.recv(timeout=1.0)
    if fc and fc.arbitration_id == 0x784:
        pci = fc.data[0]
        if (pci & 0xF0) == 0x30:  # FC frame
            st_min_raw = fc.data[2]

            # STmin interpretation per ISO-TP:
            # 0x00-0x7F = 0-127 ms
            # 0xF1-0xF9 = 100-900 µs (we'll clamp up)
            if st_min_raw <= 0x7F:
                st_min = st_min_raw / 1000.0
            elif 0xF1 <= st_min_raw <= 0xF9:
                st_min = 0.001

    # Send Consecutive Frames
    for arb_id, data in frames[1:]:
        if st_min > 0:
            time.sleep(st_min)
        dbg_tx(arb_id, data)
        bus.send(can.Message(arbitration_id=arb_id, data=data, is_extended_id=False))


# =============================================================================
#  GATEWAY IGNITION RESPONDER (DID 0x6401)
# =============================================================================

def handle_gateway_ignition(bus, msg, ignition_state):
    data = bytes(msg.data)
    if len(data) < 4:
        return

    pci = data[0]
    if (pci & 0xF0) != 0x00:
        return

    length = pci & 0x0F
    if length < 3:
        return

    sid = data[1]
    did_hi = data[2]
    did_lo = data[3]

    if sid == 0x22 and did_hi == 0x64 and did_lo == 0x01:
        payload = bytes([0x62, 0x64, 0x01, ignition_state])
        sf_pci = 0x00 | len(payload)
        out = bytes([sf_pci]) + payload
        out = out.ljust(8, b"\x00")

        dbg_tx(0x718, out)
        bus.send(can.Message(arbitration_id=0x718, data=out, is_extended_id=False))


# =============================================================================
#  ABS RESPONDER (SYNTHETIC + RAW)
# =============================================================================

def handle_abs_request(bus, msg, abs_mode, abs_dtc_list, abs_raw_frames):
    data = bytes(msg.data)
    if len(data) < 2:
        return

    pci = data[0]
    if (pci & 0xF0) != 0x00:
        return

    sid = data[1]
    if sid != 0x18:
        return

    # Synthetic ABS DTCs
    if abs_mode == "dtc" and abs_dtc_list:
        frames = build_abs_synthetic_response(abs_dtc_list)
        if len(frames) == 1:
            arb_id, d = frames[0]
            dbg_tx(arb_id, d)
            bus.send(can.Message(arbitration_id=arb_id, data=d, is_extended_id=False))
        else:
            send_isotp_multiframe(bus, frames)

    # Raw replay (BusMaster or txt)
    elif abs_mode in ("raw_busmaster", "raw_txt") and abs_raw_frames:
        for arb_id, d in abs_raw_frames:
            dbg_tx(arb_id, d)
            bus.send(can.Message(arbitration_id=arb_id, data=d, is_extended_id=False))


# =============================================================================
#  AC COMPRESSOR RESPONDER (RAW ONLY)
# =============================================================================

def handle_ac_request(bus, msg, ac_mode, ac_raw_frames):
    """
    Handle ACCompressor UDS 0x19 02 (DTC by status mask) on ID 0x7C7.
    Response on 0x7CF.
    Raw replay only for now.
    """
    data = bytes(msg.data)
    if len(data) < 3:
        return

    pci = data[0]
    if (pci & 0xF0) != 0x00:
        return

    sid = data[1]
    sub = data[2]

    if sid == 0x19 and sub == 0x02:
        if ac_mode == "raw_txt" and ac_raw_frames:
            for arb_id, d in ac_raw_frames:
                dbg_tx(arb_id, d)
                bus.send(can.Message(arbitration_id=arb_id, data=d, is_extended_id=False))


# =============================================================================
#  AIRBAG RESPONDER (SYNTHETIC + RAW)
# =============================================================================

def handle_airbag_request(bus, msg, airbag_mode, airbag_dtc_list, airbag_raw_frames):
    """
    Airbag uses UDS 0x18 on request ID 0x7C4, response on 0x7CC.
    Observed:
      - Req: 7C4 04 18 02 FF 00 00 00 00
      - Rsp: 7CC 03 7F 18 78 00 00 00 00
             7CC 05 58 01 A5 05 E0 00 00
    """
    data = bytes(msg.data)
    if len(data) < 2:
        return

    pci = data[0]
    if (pci & 0xF0) != 0x00:
        return

    sid = data[1]
    if sid != 0x18:
        return

    # Raw replay mode
    if airbag_mode in ("raw_busmaster", "raw_txt") and airbag_raw_frames:
        for arb_id, d in airbag_raw_frames:
            dbg_tx(arb_id, d)
            bus.send(can.Message(arbitration_id=arb_id, data=d, is_extended_id=False))
        return

    # Synthetic mode
    if airbag_mode == "dtc":
        frames = build_airbag_synthetic_response(airbag_dtc_list or [])
        for arb_id, d in frames:
            dbg_tx(arb_id, d)
            bus.send(can.Message(arbitration_id=arb_id, data=d, is_extended_id=False))


# =============================================================================
#  MAIN DISPATCH LOOP
# =============================================================================

def main():
    p = argparse.ArgumentParser(description="Unified Fake Coda Car Simulator")
    p.add_argument("--interface", default="kvaser")
    p.add_argument("--channel", default="0")

    # ABS modes
    p.add_argument("--abs-dtc", nargs="*", help="ABS DTCs like C2100 C102B")
    p.add_argument("--abs-raw-busmaster", help="ABS raw replay from BusMaster log")
    p.add_argument("--abs-raw-txt", help="ABS raw replay from read_dtcs_coda log")

    # ACCompressor modes (raw only)
    p.add_argument("--ac-raw-txt", help="ACCompressor raw replay from log")

    # Airbag modes
    p.add_argument("--airbag-dtc", nargs="*", help="Airbag DTCs (synthetic) like Bxxxx/Uxxxx")
    p.add_argument("--airbag-raw-busmaster", help="Airbag raw replay from BusMaster log")
    p.add_argument("--airbag-raw-txt", help="Airbag raw replay from script log")

    # Ignition state
    p.add_argument("--ignition-state", default="3")

    args = p.parse_args()

    # ABS mode selection
    abs_mode = None
    abs_dtc_list = None
    abs_raw_frames = None

    if args.abs_raw_busmaster:
        abs_mode = "raw_busmaster"
        abs_raw_frames = load_abs_raw_busmaster(args.abs_raw_busmaster)
    elif args.abs_raw_txt:
        abs_mode = "raw_txt"
        abs_raw_frames = load_abs_raw_txt(args.abs_raw_txt)
    elif args.abs_dtc:
        abs_mode = "dtc"
        abs_dtc_list = [x.strip().upper() for x in args.abs_dtc]

    # AC mode selection (raw only)
    ac_mode = None
    ac_raw_frames = None

    if args.ac_raw_txt:
        ac_mode = "raw_txt"
        ac_raw_frames = load_ac_raw_txt(args.ac_raw_txt)

    # Airbag mode selection
    airbag_mode = None
    airbag_dtc_list = None
    airbag_raw_frames = None

    if args.airbag_raw_busmaster:
        airbag_mode = "raw_busmaster"
        airbag_raw_frames = load_airbag_raw_busmaster(args.airbag_raw_busmaster)
    elif args.airbag_raw_txt:
        airbag_mode = "raw_txt"
        airbag_raw_frames = load_airbag_raw_txt(args.airbag_raw_txt)
    elif args.airbag_dtc:
        airbag_mode = "dtc"
        airbag_dtc_list = [x.strip().upper() for x in args.airbag_dtc]
    else:
        # If no airbag args, default to synthetic "no DTCs" when asked
        airbag_mode = "dtc"
        airbag_dtc_list = []

    ign_str = args.ignition_state.strip().lower()
    ignition_state = int(ign_str, 16) if ign_str.startswith("0x") else int(ign_str)

    print(f"[CONFIG] Interface={args.interface} Channel={args.channel}")
    print(f"[CONFIG] Ignition state=0x{ignition_state:02X}")

    print(f"[CONFIG] ABS mode={abs_mode}")
    if abs_mode == "dtc":
        print(f"[CONFIG] ABS synthetic DTCs={abs_dtc_list}")
    elif abs_mode in ("raw_busmaster", "raw_txt"):
        print(f"[CONFIG] ABS raw frames={len(abs_raw_frames) if abs_raw_frames else 0}")

    print(f"[CONFIG] AC mode={ac_mode}")
    if ac_mode == "raw_txt":
        print(f"[CONFIG] AC raw frames={len(ac_raw_frames) if ac_raw_frames else 0}")

    print(f"[CONFIG] Airbag mode={airbag_mode}")
    if airbag_mode == "dtc":
        print(f"[CONFIG] Airbag synthetic DTCs={airbag_dtc_list}")
    elif airbag_mode in ("raw_busmaster", "raw_txt"):
        print(f"[CONFIG] Airbag raw frames={len(airbag_raw_frames) if airbag_raw_frames else 0}")

    bus = can.interface.Bus(interface=args.interface,
                            channel=args.channel,
                            bitrate=BITRATE)

    while True:
        msg = bus.recv(timeout=1.0)
        if msg is None:
            continue

        dbg_rx(msg.arbitration_id, msg.data)

        arb = msg.arbitration_id

        if arb == 0x710:
            handle_gateway_ignition(bus, msg, ignition_state)
        elif arb == 0x784:
            handle_abs_request(bus, msg, abs_mode, abs_dtc_list, abs_raw_frames)
        elif arb == 0x7C7:
            handle_ac_request(bus, msg, ac_mode, ac_raw_frames)
        elif arb == 0x7C4:
            handle_airbag_request(bus, msg, airbag_mode, airbag_dtc_list, airbag_raw_frames)


if __name__ == "__main__":
    main()
