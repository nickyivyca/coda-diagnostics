#!/usr/bin/env python3
r"""Coda cell voltage display -- rebuild of Cell_Voltage_Display.exe.

Same visual baseline as the original tool, but portable: the original is a
Windows executable bound to Kvaser's canlib32.dll, whereas this runs wherever
Python and python-can do and talks to any dongle python-can supports.  Reading
a log needs no dongle at all.

It also draws either the original's cell placement ("app") or the corrected
physical one ("corrected") -- the original puts 44 of 104 cells in the wrong
physical position.  See README.md for the mapping and the deliberate
deviations.

Reads the 26 cell-voltage frames the BMS broadcasts on C CAN (0x000-0x019),
four 16-bit big-endian millivolt values per frame.  This is the BMS's own
diagnostic broadcast -- a different path from read_cell_voltages_and_temps.py,
which requests cells over UDS through the gateway.

Sources                      State                   Renderers
-------                      -----                   ---------
--log <paths...>  ---\                          /--- --gui      live Tkinter
   python-can          >--- PackState (104 x V) -<
--live            ---/       + max/min          \--- --render   PNG via Pillow

Typical use:

    python cell_viewer/cell_viewer.py --log capture.log --scan
    python cell_viewer/cell_viewer.py --log capture.log --at demo --render out.png
    python cell_viewer/cell_viewer.py --log capture.log --gui --speed 20
    python cell_viewer/cell_viewer.py --live --gui
"""
from __future__ import annotations

import argparse
import datetime
import glob
import heapq
import json
import os
import re
import sys
import time
from collections import Counter, deque

# Allow running by path from anywhere; layout.py is a sibling module.
_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import can  # noqa: E402
import layout  # noqa: E402  (sibling module)

N_CELLS = 104
SWEEP_IDS = list(layout.CELL_FRAME_IDS)
LAST_ID = SWEEP_IDS[-1]          # 0x19 -- the frame the original publishes on

# Logical widget geometry.  A tk Label(width=12, font=("courier new",14,"bold"))
# measures ~139 x 28 px; the original tool was screenshotted at 2x DPI.
CELL_W, CELL_H = 139, 28
FONT_PX = 19                     # 14 pt at 96 dpi
ROOT_BG = (240, 240, 240)        # Windows SystemButtonFace

# X11 colour names, which is what tk resolves against.
X11 = {
    "black": (0, 0, 0), "white": (255, 255, 255), "blue": (0, 0, 255),
    "green": (0, 255, 0), "red": (255, 0, 0), "purple": (160, 32, 240),
}


# --------------------------------------------------------------- pack state
class PackState:
    """104 cell voltages plus the original's per-sweep max/min bookkeeping.

    The original resets its running max/min when frame 0x000 arrives and
    publishes them when 0x019 arrives, so the readouts describe one sweep of
    the pack rather than an all-time extreme.  Reproduced here exactly.
    """

    def __init__(self):
        self.volts = {}          # cell number (1..104) -> volts
        self.last_seen = {}      # cell number -> frame timestamp
        self.run_max = None
        self.run_min = None
        self.max_v = None        # published at the end of each sweep
        self.min_v = None
        self.t = None
        self.sweeps = 0
        self.history = deque(maxlen=layout.SCOPE_SAMPLES)   # (t, min, max)

    @property
    def complete(self):
        return len(self.volts) == N_CELLS

    def update(self, frame):
        """Feed one CAN frame.  Returns True on the sweep-publish frame."""
        pairs = layout.decode_frame(frame.arbitration_id, frame.data)
        if not pairs:
            return False
        if frame.arbitration_id == 0:
            self.run_max = self.run_min = pairs[0][1]
        for n, v in pairs:
            self.volts[n] = v
            self.last_seen[n] = frame.timestamp
            if self.run_max is None or v > self.run_max:
                self.run_max = v
            if self.run_min is None or v < self.run_min:
                self.run_min = v
        if frame.arbitration_id == LAST_ID:
            self.max_v, self.min_v = self.run_max, self.run_min
            self.t = frame.timestamp
            self.sweeps += 1
            if self.max_v is not None and self.min_v is not None:
                self.history.append((frame.timestamp, self.min_v, self.max_v))
            return True
        return False

    def extremes(self):
        """-> (max_cell, max_v, min_cell, min_v) over the current grid."""
        if not self.volts:
            return (None, None, None, None)
        hi = max(self.volts.items(), key=lambda kv: kv[1])
        lo = min(self.volts.items(), key=lambda kv: kv[1])
        return (hi[0], hi[1], lo[0], lo[1])

    def snapshot(self):
        return dict(self.volts)


# ------------------------------------------------------------- frame sources
def expand_logs(patterns):
    """Expand globs and de-duplicate."""
    out = []
    for pat in patterns:
        hits = sorted(glob.glob(pat)) if any(c in pat for c in "*?[") else [pat]
        if not hits:
            raise SystemExit("no log files match: " + pat)
        out.extend(hits)
    seen, paths = set(), []
    for p in out:
        rp = os.path.abspath(p)
        if rp not in seen:
            seen.add(rp)
            paths.append(p)
    return paths


def _channel_key(msg):
    """Normalise a python-can channel to something hashable and comparable.

    candump logs carry the interface name ("vcan0"); .asc/.blf may carry an
    int.  Everything is compared as a string so the two behave alike.
    """
    ch = getattr(msg, "channel", None)
    return "" if ch is None else str(ch)


def detect_cell_channel(paths, sample=400000):
    """Which recorded channel carries the C CAN cell frames.

    Never assume a channel name/number -- it moves with dongle insertion
    order.  Identify by content: 8-byte frames on IDs 0x000-0x019.
    """
    counts = Counter()
    for p in paths:
        for i, m in enumerate(read_log(p)):
            if m.arbitration_id in layout.CELL_FRAME_IDS and m.dlc == 8:
                counts[_channel_key(m)] += 1
            if i >= sample:
                break
        if counts:
            break
    if not counts:
        raise SystemExit("no cell frames (IDs 0x000-0x019, DLC 8) in the sample; "
                         "this capture probably does not include C CAN")
    return counts.most_common(1)[0][0]


# candump's CONSOLE output, which is not the same as its log format:
#     (2026-09-09 23:33:41.381427)  can0  000   [8]  FF FF FF FF FF FF FF FF
# python-can reads only the log format, "(1234567890.123456) can0 000#FFFF..",
# and fails on this one with an unpack error, so it is handled here.
_CONSOLE_RE = re.compile(
    r"^\s*\((?P<ts>[^)]+)\)\s+"              # (timestamp)
    r"(?P<iface>\S+)\s+"                     # can0
    r"(?P<id>[0-9A-Fa-f]+)\s+"               # 000
    r"\[(?P<dlc>\d+)\]\s*"                   # [8]
    r"(?P<data>(?:[0-9A-Fa-f]{2}\s*)*)$"     # FF FF ...
)


def _console_timestamp(text):
    """-> epoch seconds, from either a date-time or a bare number."""
    text = text.strip()
    for fmt in ("%Y-%m-%d %H:%M:%S.%f", "%Y-%m-%d %H:%M:%S"):
        try:
            return datetime.datetime.strptime(text, fmt).timestamp()
        except ValueError:
            pass
    try:
        return float(text)          # absolute epoch, or a relative offset
    except ValueError:
        return None


def _read_console(path):
    """Yield Messages from a candump console capture."""
    with open(path, "r", encoding="utf-8", errors="replace") as fh:
        for line in fh:
            m = _CONSOLE_RE.match(line)
            if not m:
                continue
            ts = _console_timestamp(m.group("ts"))
            if ts is None:
                continue
            dlc = int(m.group("dlc"))
            data = bytes.fromhex(m.group("data").replace(" ", ""))
            arb = m.group("id")
            yield can.Message(timestamp=ts, arbitration_id=int(arb, 16),
                              data=data[:dlc], dlc=dlc,
                              is_extended_id=len(arb) > 3,
                              channel=m.group("iface"))


def _looks_like_console(path, sniff=40):
    """True if the first readable line is candump console output."""
    try:
        with open(path, "r", encoding="utf-8", errors="replace") as fh:
            for _ in range(sniff):
                line = fh.readline()
                if not line:
                    break
                if line.strip():
                    return bool(_CONSOLE_RE.match(line))
    except OSError:
        pass
    return False


def read_log(path):
    """Yield python-can Messages from one log file.

    candump console captures are read directly; everything else goes to
    can.LogReader, which dispatches on the extension and so covers candump
    .log, Vector .asc, SavvyCAN .csv and .blf.  Messages without a timestamp
    are skipped rather than allowed to poison the sweep timeline.
    """
    if _looks_like_console(path):
        yield from _read_console(path)
        return
    try:
        with can.LogReader(path) as reader:
            for msg in reader:
                if msg.timestamp is None:
                    continue
                yield msg
    except ValueError as exc:
        raise SystemExit(
            "could not parse %s as a CAN log (%s).\n"
            "Supported: candump console output, candump .log, Vector .asc, "
            "SavvyCAN .csv and .blf. A .log in some other format -- BUSMASTER, "
            "for instance -- has to be converted first." % (path, exc))


def frame_stream(paths):
    """Yield messages from one or more logs in timestamp order."""
    if len(paths) == 1:
        yield from read_log(paths[0])
    else:
        yield from heapq.merge(*(read_log(p) for p in paths),
                               key=lambda m: m.timestamp)


# ------------------------------------------------------------------ sweeps
def collect_sweeps(paths, channel=None, limit=None, progress=True,
                   partial=False):
    """One streaming pass -> a list of complete-sweep snapshots.

    Cell frames are sparse relative to total traffic, so a whole session
    yields only a few hundred sweeps; keeping every snapshot costs nothing
    and means --scan, --at and --render all share a single pass.

    With *partial* true, a sweep missing some of the 104 cells is kept
    instead of discarded, and a sweep still open when the stream ends is
    flushed.  That is for short captures that begin or end mid-sweep -- a
    pasted console dump, say -- where no run of 0x000..0x019 is intact.
    Cells never seen keep the original's ``NN_k`` placeholder, and the
    max/min readouts describe only the cells that were captured.
    """
    if channel is None:
        channel = detect_cell_channel(paths)
        if progress:
            print("[channel] cell frames detected on channel %r" % channel)
    state = PackState()
    sweeps = []
    t0 = time.time()
    nframes = 0
    for frame in frame_stream(paths):
        nframes += 1
        if progress and nframes % 2000000 == 0:
            print("[scan] %.0fM frames, %d sweeps, %.0fs"
                  % (nframes / 1e6, len(sweeps), time.time() - t0))
        if (_channel_key(frame) != channel
                or frame.arbitration_id not in layout.CELL_FRAME_IDS):
            continue
        if state.update(frame) and (state.complete or partial):
            sweeps.append({"t": state.t, "min": state.min_v, "max": state.max_v,
                           "volts": state.snapshot(),
                           "cells": len(state.volts)})
            if limit and len(sweeps) >= limit:
                break
    if partial and state.volts and not sweeps:
        # The stream ended without a publish frame (0x019).  Publish what the
        # open sweep holds so a capture that stops mid-pack still renders.
        sweeps.append({"t": max(state.last_seen.values()),
                       "min": state.run_min, "max": state.run_max,
                       "volts": state.snapshot(), "cells": len(state.volts)})
    if progress:
        print("[scan] %d frames, %d sweeps, %.1fs"
              % (nframes, len(sweeps), time.time() - t0))
        if partial:
            short = [s for s in sweeps if s.get("cells", N_CELLS) < N_CELLS]
            if short:
                print("[scan] %d of %d sweeps are partial (fewest %d/%d cells)"
                      % (len(short), len(sweeps),
                         min(s["cells"] for s in short), N_CELLS))
    return sweeps


def load_or_collect(args):
    cache = args.cache
    if cache and os.path.exists(cache) and not args.refresh:
        with open(cache, "r", encoding="utf-8") as fh:
            raw = json.load(fh)
        for s in raw:
            s["volts"] = {int(k): v for k, v in s["volts"].items()}
        print("[cache] loaded %d sweeps from %s" % (len(raw), cache))
        return raw
    sweeps = collect_sweeps(expand_logs(args.log), channel=args.channel,
                            limit=args.limit, partial=args.partial)
    if cache:
        os.makedirs(os.path.dirname(os.path.abspath(cache)), exist_ok=True)
        with open(cache, "w", encoding="utf-8") as fh:
            json.dump(sweeps, fh)
        print("[cache] wrote %d sweeps to %s" % (len(sweeps), cache))
    return sweeps


# ------------------------------------------------------------------- scan
def describe(sweep, mapping):
    pos = layout.cell_positions(mapping)
    volts = sweep["volts"]
    hi = max(volts.items(), key=lambda kv: kv[1])
    lo = min(volts.items(), key=lambda kv: kv[1])
    return {
        "t": sweep["t"],
        "max_cell": hi[0], "max_v": hi[1], "max_rc": pos[hi[0]],
        "min_cell": lo[0], "min_v": lo[1], "min_rc": pos[lo[0]],
        "spread": hi[1] - lo[1],
    }


def cmd_scan(sweeps, mapping, top=5):
    if not sweeps:
        print("no complete sweeps found")
        return
    rows = [describe(s, mapping) for s in sweeps]
    span = (rows[0]["t"], rows[-1]["t"])
    print("")
    print("%d complete sweeps, t = %.1f .. %.1f s (%.1f min)"
          % (len(rows), span[0], span[1], (span[1] - span[0]) / 60.0))

    def show(title, keyfn, reverse=True):
        print("")
        print("-- " + title + " --")
        print("%10s  %9s  %22s  %22s" % ("t (s)", "spread mV", "max", "min"))
        for r in sorted(rows, key=keyfn, reverse=reverse)[:top]:
            print("%10.2f  %9.0f  Cell%-3d %.3fV (r%d,c%d)  Cell%-3d %.3fV (r%d,c%d)"
                  % (r["t"], r["spread"] * 1000,
                     r["max_cell"], r["max_v"], r["max_rc"][0], r["max_rc"][1],
                     r["min_cell"], r["min_v"], r["min_rc"][0], r["min_rc"][1]))

    show("largest spread", lambda r: r["spread"])
    show("highest cell", lambda r: r["max_v"])
    show("lowest cell", lambda r: r["min_v"], reverse=False)

    # Cells in ODD display columns land in the same place under both mappings,
    # so only an EVEN-column extreme demonstrates the correction on screen.
    demo = [r for r in rows
            if r["max_rc"][1] % 2 == 0 or r["min_rc"][1] % 2 == 0]
    print("")
    print("-- best demo instants (extreme cell in an EVEN column: it moves "
          "between mappings) --")
    if not demo:
        print("  none -- every sweep has both extremes in odd columns")
    else:
        for r in sorted(demo, key=lambda r: r["spread"], reverse=True)[:top]:
            which = "max" if r["max_rc"][1] % 2 == 0 else "min"
            rc = r[which + "_rc"]
            print("%10.2f  spread %4.0f mV  %s = Cell%d at (r%d,c%d) %.3fV"
                  % (r["t"], r["spread"] * 1000, which, r[which + "_cell"],
                     rc[0], rc[1], r[which + "_v"]))


def pick_sweep(sweeps, at, mapping):
    """--at accepts a timestamp in seconds or a keyword."""
    if not sweeps:
        raise SystemExit("no complete sweeps to render")
    rows = [describe(s, mapping) for s in sweeps]
    if at in ("max-spread", "max-cell", "min-cell", "demo"):
        if at == "max-spread":
            i = max(range(len(rows)), key=lambda i: rows[i]["spread"])
        elif at == "max-cell":
            i = max(range(len(rows)), key=lambda i: rows[i]["max_v"])
        elif at == "min-cell":
            i = min(range(len(rows)), key=lambda i: rows[i]["min_v"])
        else:
            cand = [i for i, r in enumerate(rows)
                    if r["max_rc"][1] % 2 == 0 or r["min_rc"][1] % 2 == 0]
            if not cand:
                # An even-column extreme is what makes the two mappings differ
                # visibly, so "demo" prefers one -- but plenty of captures have
                # none, and refusing to render at all would be useless.  Fall
                # back to the widest spread, which is what demo means anyway.
                cand = list(range(len(rows)))
            i = max(cand, key=lambda i: rows[i]["spread"])
        return i, sweeps[i], rows[i]
    try:
        t = float(at)
    except ValueError:
        raise SystemExit("--at: expected seconds or a keyword, got " + repr(at))
    i = min(range(len(sweeps)), key=lambda i: abs(sweeps[i]["t"] - t))
    return i, sweeps[i], rows[i]


# ---------------------------------------------------------------- PIL render
def _font(px):
    from PIL import ImageFont
    fonts = os.path.join(os.environ.get("WINDIR", "C:/Windows"), "Fonts")
    for name in ("courbd.ttf", "cour.ttf"):
        p = os.path.join(fonts, name)
        if os.path.exists(p):
            return ImageFont.truetype(p, px)
    return ImageFont.load_default()


def _bevel(draw, x, y, w, h, bg, raised, bw):
    light = tuple(int(c + (255 - c) * 0.45) for c in bg)
    dark = tuple(int(c * 0.55) for c in bg)
    tl, br = (light, dark) if raised else (dark, light)
    draw.rectangle([x, y, x + w - 1, y + h - 1], fill=bg)
    for i in range(bw):
        draw.line([(x + i, y + i), (x + w - 1 - i, y + i)], fill=tl)
        draw.line([(x + i, y + i), (x + i, y + h - 1 - i)], fill=tl)
        draw.line([(x + i, y + h - 1 - i), (x + w - 1 - i, y + h - 1 - i)], fill=br)
        draw.line([(x + w - 1 - i, y + i), (x + w - 1 - i, y + h - 1 - i)], fill=br)


def _cell_text(cell, volts):
    """The original prints str(mv/1000); un-seen cells keep their placeholder."""
    if cell in volts:
        return str(volts[cell])
    can_id, slot = divmod(cell - 1, 4)
    return "%02d_%d" % (can_id, slot)


def render_png(volts, mapping, out_path, scale=2, max_v=None, min_v=None,
               caption=None):
    from PIL import Image, ImageDraw
    grid = layout.build(mapping)
    win_w, win_h = layout.WINDOW["31kWh"]
    cw, ch = CELL_W * scale, CELL_H * scale
    bw = 2 * scale
    cap_h = int(22 * scale) if caption else 0
    img = Image.new("RGB", (win_w * scale, win_h * scale + cap_h), ROOT_BG)
    draw = ImageDraw.Draw(img)
    font = _font(FONT_PX * scale)

    def box(tk_row, tk_col, text, fg, bg, raised=False):
        x, y = tk_col * cw, tk_row * ch
        _bevel(draw, x, y, cw, ch, X11[bg], raised, bw)
        tb = draw.textbbox((0, 0), text, font=font)
        draw.text((x + (cw - (tb[2] - tb[0])) / 2 - tb[0],
                   y + (ch - (tb[3] - tb[1])) / 2 - tb[1]),
                  text, font=font, fill=X11[fg])

    for (row, col), (cell, q) in grid.items():
        box(row - 1, col - 1, _cell_text(cell, volts),
            layout.FG_COLORS[q], layout.BG_COLORS[q])

    # tk grid rows 12 and 13 hold no widgets, so they collapse to zero height
    # and the readouts sit directly under the grid.
    if max_v is None:
        max_v = max(volts.values()) if volts else 0.0
    if min_v is None:
        min_v = min(volts.values()) if volts else 0.0
    r_max, r_min = layout.NROW_31, layout.NROW_31 + 1
    box(r_max, 0, "Maximum", layout.MAX_COLORS["fg"], layout.MAX_COLORS["bg"],
        raised=True)
    box(r_max, 1, str(max_v), layout.MAX_COLORS["fg"], layout.MAX_COLORS["bg"])
    box(r_min, 0, "Minimum", layout.MIN_COLORS["fg"], layout.MIN_COLORS["bg"],
        raised=True)
    box(r_min, 1, str(min_v), layout.MIN_COLORS["fg"], layout.MIN_COLORS["bg"])

    if caption:
        draw.rectangle([0, win_h * scale, img.width, img.height], fill=(0, 0, 0))
        draw.text((6 * scale, win_h * scale + 3 * scale), caption,
                  font=_font(int(13 * scale)), fill=(255, 255, 255))

    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
    img.save(out_path)
    return out_path


def render_scope_png(history, out_path, scale=2, mode="fixed"):
    """The second window.  See --scope-mode; "original" reproduces its bug."""
    from PIL import Image, ImageDraw
    w, h = layout.SCOPE_WINDOW
    img = Image.new("RGB", (w * scale, h * scale), X11["white"])
    draw = ImageDraw.Draw(img)
    pts = list(history)
    if not pts:
        img.save(out_path)
        return out_path
    if mode == "original":
        # Verbatim from the decompiled source: scales with canvas_height (400,
        # window 1's height) on a canvas only 200 tall, so nothing below 3.4 V
        # is ever visible.
        hh = layout.WINDOW["31kWh"][1]

        def ypx(v):
            return int(hh - hh * ((v - layout.SCOPE_V_MIN) / layout.SCOPE_V_SPAN))
        lo = hi = None
    else:
        lo = min(min(p[1] for p in pts), min(p[2] for p in pts))
        hi = max(max(p[1] for p in pts), max(p[2] for p in pts))
        pad = max((hi - lo) * 0.1, 0.005)
        lo, hi = lo - pad, hi + pad

        def ypx(v):
            return int(h - (v - lo) / (hi - lo) * h)
    for i, (_t, mn, mx) in enumerate(pts):
        x = (i * 6 + 3) * scale
        r = 3 * scale
        for v, colour in ((mn, "purple"), (mx, "red")):
            y = ypx(v) * scale
            draw.ellipse([x - r, y - r, x + r, y + r],
                         fill=X11[colour], outline=X11[colour])
    if mode != "original":
        f = _font(int(11 * scale))
        draw.text((4 * scale, 2 * scale), "%.3fV" % hi, font=f, fill=(0, 0, 0))
        draw.text((4 * scale, (h - 16) * scale), "%.3fV" % lo, font=f,
                  fill=(0, 0, 0))
    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
    img.save(out_path)
    return out_path


# --------------------------------------------------------------------- gui
def run_gui(source, mapping, speed=1.0, scope_mode="fixed", scope=False):
    """Real tk widgets, placed from layout.build() -- the faithful renderer.

    The original opened a second window plotting per-sweep min/max over
    time.  It is off here unless *scope* is set: it is a separate window to
    manage for something the grid's own Maximum/Minimum readouts already
    say, and the original's own scaling left it blank anyway.
    """
    import tkinter as tk

    try:
        root = tk.Tk()
    except tk.TclError as exc:
        # The display is the default action, so this is the first thing a
        # headless user hits; a bare Tcl error would not explain itself.
        raise SystemExit(
            "cannot open a window (%s).\n"
            "This needs a desktop session. On a headless machine use --scan "
            "for a summary or --render OUT.png to write an image." % exc)
    root.title("%s  [%s]" % (layout.TITLE, mapping))
    w, h = layout.WINDOW["31kWh"]
    root.geometry("%dx%d+%d+%d" % (w, h, layout.WINDOW_POS[0],
                                   layout.WINDOW_POS[1]))
    tkfont = layout.FONT

    cells = {}
    for (row, col), (cell, q) in layout.build(mapping).items():
        var = tk.StringVar()
        var.set(_cell_text(cell, {}))
        tk.Label(root, textvariable=var, fg=layout.FG_COLORS[q],
                 bg=layout.BG_COLORS[q], font=tkfont,
                 relief=tk.SUNKEN, width=layout.CELL_WIDTH_CHARS
                 ).grid(row=row - 1, column=col - 1)
        cells[cell] = var

    var_max, var_min = tk.StringVar(), tk.StringVar()
    var_max.set("max")
    var_min.set("min")
    r_max, r_min = layout.MAXMIN_ROWS
    tk.Label(root, text="Maximum", font=tkfont, relief=tk.RAISED,
             width=layout.CELL_WIDTH_CHARS, fg=layout.MAX_COLORS["fg"],
             bg=layout.MAX_COLORS["bg"]).grid(row=r_max, column=0)
    tk.Label(root, textvariable=var_max, font=tkfont, relief=tk.SUNKEN,
             width=layout.CELL_WIDTH_CHARS, fg=layout.MAX_COLORS["fg"],
             bg=layout.MAX_COLORS["bg"]).grid(row=r_max, column=1)
    tk.Label(root, text="Minimum", font=tkfont, relief=tk.RAISED,
             width=layout.CELL_WIDTH_CHARS, fg=layout.MIN_COLORS["fg"],
             bg=layout.MIN_COLORS["bg"]).grid(row=r_min, column=0)
    tk.Label(root, textvariable=var_min, font=tkfont, relief=tk.SUNKEN,
             width=layout.CELL_WIDTH_CHARS, fg=layout.MIN_COLORS["fg"],
             bg=layout.MIN_COLORS["bg"]).grid(row=r_min, column=1)

    sw, sh = layout.SCOPE_WINDOW
    hist = deque(maxlen=layout.SCOPE_SAMPLES)
    canvas = None
    if scope:
        scope_win = tk.Toplevel(root)
        scope_win.title(layout.SCOPE_TITLE)
        scope_win.geometry("%dx%d+%d+%d" % (sw, sh, layout.SCOPE_POS[0],
                                            layout.SCOPE_POS[1]))
        canvas = tk.Canvas(scope_win, width=sw, height=sh, bg="white")
        canvas.pack()

    def draw_scope():
        if canvas is None:
            return
        canvas.delete("all")
        if not hist:
            return
        if scope_mode == "original":
            hh = layout.WINDOW["31kWh"][1]

            def ypx(v):
                return int(hh - hh * ((v - layout.SCOPE_V_MIN)
                                      / layout.SCOPE_V_SPAN))
        else:
            lo = min(min(p[0] for p in hist), min(p[1] for p in hist))
            hi = max(max(p[0] for p in hist), max(p[1] for p in hist))
            pad = max((hi - lo) * 0.1, 0.005)
            lo, hi = lo - pad, hi + pad

            def ypx(v):
                return int(sh - (v - lo) / (hi - lo) * sh)
        for i, (mn, mx) in enumerate(hist):
            x = i * 6 + 3
            for v, colour in ((mn, "purple"), (mx, "red")):
                y = ypx(v)
                canvas.create_oval(x, y, x, y, fill=colour, outline=colour,
                                   width=6)

    def apply(volts, mx, mn):
        for cell, var in cells.items():
            if cell in volts:
                var.set(str(volts[cell]))
        var_max.set(str(mx))
        var_min.set(str(mn))
        hist.append((mn, mx))
        draw_scope()

    def step():
        try:
            volts, mx, mn, dt = next(source)
        except StopIteration:
            root.title(root.title() + "  -- end of log")
            return
        apply(volts, mx, mn)
        root.after(max(1, int(dt * 1000 / speed)), step)

    root.after(10, step)
    root.mainloop()


def sweep_source(sweeps, loop=True):
    """-> generator of (volts, max, min, seconds-until-next-sweep).

    Loops by default: a capture is usually far shorter than you want to look
    at it for, and a display that stops after a few seconds is less useful
    than one that keeps running.  The gap after the last sweep doubles as the
    pause before it wraps.
    """
    while True:
        for i, s in enumerate(sweeps):
            dt = (sweeps[i + 1]["t"] - s["t"]) if i + 1 < len(sweeps) else 0.5
            yield s["volts"], s["max"], s["min"], max(dt, 0.0)
        if not loop:
            return


def live_source(channel, interface, bitrate, partial=False):
    """-> generator of (volts, max, min, dt) from a live bus.

    A python-can Message already carries .timestamp/.arbitration_id/.data, so
    it is fed to PackState directly.  When interface or channel is left unset
    the repo's shared dongle detection picks them, so this tool finds the same
    hardware as read_dtcs_coda.py.  That import is deferred because it pulls in
    the isotp stack, which log replay has no use for.
    """
    if interface is None or channel is None:
        sys.path.insert(0, os.path.dirname(_HERE))
        from read_dtcs_coda import detect_can_interface
        found_if, found_ch = detect_can_interface()
        interface = interface or found_if
        channel = channel if channel is not None else found_ch
        print("[live] using interface=%s channel=%s" % (interface, channel))

    bus = can.Bus(channel=channel, interface=interface, bitrate=bitrate)
    state = PackState()
    # The generator is normally abandoned rather than exhausted (the GUI just
    # stops pulling), so shut the bus down on close/GC as well as on error --
    # a Kvaser channel left open warns and stays claimed until the process dies.
    try:
        while True:
            msg = bus.recv(1.0)
            if msg is None:
                continue
            if state.update(msg) and (state.complete or partial):
                yield state.snapshot(), state.max_v, state.min_v, 0.0
    finally:
        bus.shutdown()


# --------------------------------------------------------------------- main
def main(argv=None):
    ap = argparse.ArgumentParser(
        description="Coda cell voltage display (rebuild of "
                    "Cell_Voltage_Display.exe)")
    src = ap.add_argument_group("source")
    src.add_argument("--log", nargs="+", metavar="PATH",
                     help="log file(s); globs allowed, T-chunks are merged")
    src.add_argument("--live", action="store_true",
                     help="capture from a live bus instead of a log")
    src.add_argument("--interface", default=None,
                     help="CAN interface for --live (default: auto-detect, "
                          "same detection as read_dtcs_coda.py)")
    src.add_argument("--channel", default=None,
                     help="with --live, the CAN channel; with --log, which "
                          "recorded channel carries C CAN (default: the one "
                          "carrying IDs 0x000-0x019, found by content)")
    src.add_argument("--bitrate", type=int, default=500000)
    src.add_argument("--limit", type=int, help="stop after N complete sweeps")
    src.add_argument("--partial", action="store_true",
                     help="keep sweeps missing some of the 104 cells, for "
                          "captures that start or end mid-pack; un-seen cells "
                          "render as their NN_k placeholder")
    src.add_argument("--cache", metavar="JSON",
                     help="reuse/save the sweep snapshots from a scan")
    src.add_argument("--refresh", action="store_true",
                     help="ignore an existing --cache and rescan")

    act = ap.add_argument_group("action")
    act.add_argument("--scan", action="store_true",
                     help="report interesting instants and exit")
    act.add_argument("--at", metavar="T",
                     help="instant to render: seconds, or one of "
                          "max-spread / max-cell / min-cell / demo")
    act.add_argument("--render", metavar="OUT.png", help="write a PNG")
    act.add_argument("--render-scope", metavar="OUT.png",
                     help="write the second (history) window as a PNG")
    act.add_argument("--gui", action="store_true",
                     help="Tkinter display (the default when no other action "
                          "is given)")
    act.add_argument("--no-loop", dest="loop", action="store_false",
                     help="stop at the end of the log instead of looping")
    act.add_argument("--scope", action="store_true",
                     help="also open the original's second window, plotting "
                          "per-sweep min/max over time")

    opt = ap.add_argument_group("options")
    opt.add_argument("--mapping", default="corrected",
                     choices=["corrected", "app", "both"],
                     help="cell placement (default: corrected)")
    opt.add_argument("--scale", type=int, default=2,
                     help="PNG scale factor; 2 matches the original tool's screenshots")
    opt.add_argument("--scope-mode", default="fixed",
                     choices=["fixed", "original"],
                     help="'original' reproduces the original tool's broken "
                          "scaling; 'fixed' autoranges (default)")
    opt.add_argument("--speed", type=float, default=1.0, help="GUI replay speed")
    opt.add_argument("--top", type=int, default=5, help="rows per --scan table")
    args = ap.parse_args(argv)

    cache_hit = bool(args.cache) and os.path.exists(args.cache) and not args.refresh
    if not args.log and not args.live and not cache_hit:
        ap.error("one of --log or --live is required (or an existing --cache)")

    gui_mapping = "corrected" if args.mapping == "both" else args.mapping

    # The display is the default action; --scan and the renderers opt out of
    # it, and --gui forces it back on alongside them.
    rendering = bool(args.render or args.render_scope)
    show_gui = args.gui or not (args.scan or rendering)

    if args.live:
        if not show_gui:
            ap.error("--live drives the display; --scan and --render read a "
                     "log, so pass --log for those")
        run_gui(live_source(args.channel, args.interface, args.bitrate,
                            partial=args.partial),
                gui_mapping, args.speed, args.scope_mode, scope=args.scope)
        return 0

    sweeps = load_or_collect(args)

    if args.scan:
        cmd_scan(sweeps, gui_mapping, top=args.top)

    if rendering:
        _render(args, sweeps, gui_mapping)

    if show_gui:
        run_gui(sweep_source(sweeps, loop=args.loop),
                gui_mapping, args.speed, args.scope_mode, scope=args.scope)
    return 0


def _render(args, sweeps, gui_mapping):
    """--render / --render-scope."""
    idx, sweep, row = pick_sweep(sweeps, args.at or "demo", "corrected")
    print("")
    print("rendering sweep %d at t = %.2f s   max Cell%d %.3fV   "
          "min Cell%d %.3fV   spread %.0f mV"
          % (idx, sweep["t"], row["max_cell"], row["max_v"],
             row["min_cell"], row["min_v"], row["spread"] * 1000))

    # Verification step 2 from the handoff: the readouts the original computes
    # per sweep must equal the extremes of the grid it draws.
    if (abs(sweep["max"] - row["max_v"]) > 1e-9
            or abs(sweep["min"] - row["min_v"]) > 1e-9):
        print("  WARNING: sweep readout (%s/%s) != grid extremes (%s/%s)"
              % (sweep["max"], sweep["min"], row["max_v"], row["min_v"]))
    else:
        print("  readouts agree with the grid extremes")

    if args.render:
        mappings = (["app", "corrected"] if args.mapping == "both"
                    else [args.mapping])
        base, ext = os.path.splitext(args.render)
        for m in mappings:
            out = "%s_%s%s" % (base, m, ext) if len(mappings) > 1 else args.render
            pos = layout.cell_positions(m)
            cap = ("t=%.2fs  mapping=%s  max Cell%d %.3fV at r%dc%d  "
                   "min Cell%d %.3fV at r%dc%d"
                   % (sweep["t"], m, row["max_cell"], row["max_v"],
                      pos[row["max_cell"]][0], pos[row["max_cell"]][1],
                      row["min_cell"], row["min_v"],
                      pos[row["min_cell"]][0], pos[row["min_cell"]][1]))
            render_png(sweep["volts"], m, out, scale=args.scale,
                       max_v=sweep["max"], min_v=sweep["min"], caption=cap)
            print("  wrote " + out)

    if args.render_scope:
        lo = max(0, idx - layout.SCOPE_SAMPLES + 1)
        hist = [(s["t"], s["min"], s["max"]) for s in sweeps[lo:idx + 1]]
        render_scope_png(hist, args.render_scope, scale=args.scale,
                         mode=args.scope_mode)
        print("  wrote %s (scope-mode=%s)" % (args.render_scope, args.scope_mode))


if __name__ == "__main__":
    sys.exit(main())
