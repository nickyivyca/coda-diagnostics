"""
Fake Coda BMS cell-voltage broadcaster.

Emits what the BMS puts out continuously on C CAN: 26 frames, IDs 0x000-0x019,
four 16-bit big-endian millivolt values per frame, 104 cells total, sent as one
back-to-back burst that repeats at the sweep period.

This is the transmit side for testing ../cell_viewer/ without a vehicle. The
cell viewer's --live path opens a real CAN channel, so nothing in
test_cell_viewer.py can cover it; point this at one dongle and the viewer at
another wired back to back:

    python playback/coda_fake_cells.py --interface pcan --channel PCAN_USBBUS1
    python cell_viewer/cell_viewer.py --live --interface kvaser --channel 0 --gui

A single dongle also works against a Linux vcan pair, and --dry-run builds the
frames and prints them without opening any hardware at all.

Note this is a fixed synthetic profile, not a replay of real pack behaviour --
it proves the transport and decoding work end to end, not that the display is
right about any particular pack.
"""
import argparse
import time

import can

BITRATE = 500000

N_CELLS = 104
CELL_IDS = range(0x000, 0x01A)      # the 26 cell frames


def pack_profile(base_mv=3325, low_cell=None, low_mv=3243, spread=6):
    """
    Build {cell_number: millivolts} for a resting LFP pack.

    Cells sit within a few mV of each other, which is what a real pack at rest
    looks like. Pass low_cell to drive one cell well below the rest so the
    Minimum readout and its grid position are obvious at a glance.
    """
    volts = {}
    for n in range(1, N_CELLS + 1):
        volts[n] = base_mv + (n % (spread + 1)) - spread // 2
    if low_cell:
        if not 1 <= low_cell <= N_CELLS:
            raise SystemExit("--low-cell must be 1..%d" % N_CELLS)
        volts[low_cell] = low_mv
    return volts


def frames_for(volts):
    """Build [(can_id, 8 data bytes)] for one full sweep of the pack."""
    out = []
    for can_id in CELL_IDS:
        data = bytearray()
        for k in range(4):
            mv = volts[4 * can_id + k + 1]
            data += bytes([(mv >> 8) & 0xFF, mv & 0xFF])
        out.append((can_id, bytes(data)))
    return out


def main():
    p = argparse.ArgumentParser(description="Fake Coda BMS cell-voltage broadcaster")
    p.add_argument("--interface", default="kvaser")
    p.add_argument("--channel", default="0")
    p.add_argument("--bitrate", type=int, default=BITRATE)
    p.add_argument("--period", type=float, default=0.15,
                   help="seconds between sweeps; 0.15 is the driving cadence, "
                        "a plugged-in car can be as slow as 14 s")
    p.add_argument("--duration", type=float, default=30.0,
                   help="seconds to transmit for; 0 means until interrupted")
    p.add_argument("--low-cell", type=int, default=None,
                   help="drive this cell low so the Minimum readout is obvious")
    p.add_argument("--dry-run", action="store_true",
                   help="build and print the frames; open no hardware")
    a = p.parse_args()

    volts = pack_profile(low_cell=a.low_cell)
    frames = frames_for(volts)
    lo = min(volts.items(), key=lambda kv: kv[1])
    hi = max(volts.items(), key=lambda kv: kv[1])
    print("profile: %d cells, min Cell%d %d mV, max Cell%d %d mV"
          % (len(volts), lo[0], lo[1], hi[0], hi[1]))
    print("sweep  : %d frames, IDs 0x000-0x019" % len(frames))

    if a.dry_run:
        for can_id, data in frames:
            print("  %03X  %s" % (can_id, data.hex(" ").upper()))
        return 0

    channel = a.channel
    try:
        channel = int(channel)          # kvaser wants an int, pcan a string
    except ValueError:
        pass

    bus = can.interface.Bus(interface=a.interface, channel=channel,
                            bitrate=a.bitrate)
    print("tx on %s:%s at %d bps, sweep every %.2f s%s"
          % (a.interface, channel, a.bitrate, a.period,
             (", for %.0f s" % a.duration) if a.duration else ", until Ctrl-C"))

    sent = sweeps = 0
    t_end = (time.time() + a.duration) if a.duration else None
    try:
        while t_end is None or time.time() < t_end:
            t0 = time.time()
            for can_id, data in frames:
                bus.send(can.Message(arbitration_id=can_id, data=data,
                                     is_extended_id=False))
                sent += 1
            sweeps += 1
            slack = a.period - (time.time() - t0)
            if slack > 0:
                time.sleep(slack)
    except KeyboardInterrupt:
        print("interrupted")
    finally:
        try:
            bus.shutdown()
        except Exception:
            pass
    print("sent %d frames in %d sweeps" % (sent, sweeps))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
