"""Tests for cell_viewer/ -- decoding, mapping, log replay and PNG rendering.

Runs anywhere: no vehicle, no CAN hardware, no vcan and no display. The
capture is synthesised into a candump log in a tmp dir, which is also what
cell_viewer reads in normal use (can.LogReader dispatches on extension).

The Tkinter display is deliberately not covered here -- it needs a desktop
session, and headless CI does not have one.
"""
import os
import subprocess
import sys

import can
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
VIEWER_DIR = os.path.join(HERE, "cell_viewer")
sys.path.insert(0, VIEWER_DIR)
sys.path.insert(0, os.path.join(HERE, "playback"))

import cell_viewer as cv  # noqa: E402
import coda_fake_cells as fake  # noqa: E402
import layout  # noqa: E402

CHANNEL = "vcan0"
N_CELLS = 104


def _volts_for(cell):
    """A deterministic, cell-dependent voltage in a plausible LFP range."""
    return round(3.250 + (cell % 23) * 0.004, 3)


def _write_log(path, cells, channel=CHANNEL, t0=1000.0, noise_ids=()):
    """Write one sweep covering `cells` as a candump log."""
    with can.CanutilsLogWriter(str(path), channel=channel) as w:
        t = t0
        for can_id in layout.CELL_FRAME_IDS:
            block = [4 * can_id + k + 1 for k in range(4)]
            if not any(c in cells for c in block):
                continue
            data = bytearray()
            for cell in block:
                mv = int(round(_volts_for(cell) * 1000)) if cell in cells else 0
                data += bytes([(mv >> 8) & 0xFF, mv & 0xFF])
            w.on_message_received(
                can.Message(timestamp=t, arbitration_id=can_id,
                            data=bytes(data), is_extended_id=False,
                            channel=channel))
            t += 0.001
        for nid in noise_ids:
            w.on_message_received(
                can.Message(timestamp=t, arbitration_id=nid,
                            data=bytes(8), is_extended_id=False,
                            channel="can9"))
            t += 0.001


# --------------------------------------------------------------- decoding
def test_decode_frame_cell_numbering():
    """0x000 carries cells 1-4; 0x019 carries cells 101-104."""
    assert layout.decode_frame(0x000, bytes(8))[0][0] == 1
    assert layout.decode_frame(0x000, bytes(8))[-1][0] == 4
    assert layout.decode_frame(0x019, bytes(8))[-1][0] == N_CELLS


def test_decode_frame_is_big_endian_millivolts():
    # 0x0CFD = 3325 mV
    pairs = layout.decode_frame(0x00C, bytes([0x0C, 0xFD] * 4))
    assert pairs[0] == (49, 3.325)


def test_decode_frame_rejects_non_cell_ids():
    assert layout.decode_frame(0x200, bytes(8)) == []


# ---------------------------------------------------------------- mapping
def test_both_mappings_cover_all_cells():
    for mapping in ("app", "corrected"):
        grid = layout.build(mapping)
        assert len(grid) == N_CELLS
        assert sorted(n for n, _q in grid.values()) == list(range(1, N_CELLS + 1))


def test_corrected_moves_exactly_the_even_columns():
    """The correction mirrors even display columns only -- 44 cells move."""
    app = layout.cell_positions("app")
    cor = layout.cell_positions("corrected")
    moved = [n for n in app if app[n] != cor[n]]
    assert len(moved) == 44
    # No cell ever changes column, only its row within its own module.
    for n in moved:
        assert app[n][1] == cor[n][1]
        assert cor[n][1] % 2 == 0


# ------------------------------------------------------------- log replay
def test_full_sweep_from_log(tmp_path):
    log = tmp_path / "full.log"
    _write_log(log, set(range(1, N_CELLS + 1)))
    sweeps = cv.collect_sweeps([str(log)], progress=False)
    assert len(sweeps) == 1
    assert len(sweeps[0]["volts"]) == N_CELLS
    assert sweeps[0]["volts"][49] == _volts_for(49)


def test_channel_detected_by_content_not_name(tmp_path):
    """Cell frames are on vcan0; unrelated traffic is on can9."""
    log = tmp_path / "mixed.log"
    _write_log(log, set(range(1, N_CELLS + 1)), noise_ids=(0x500, 0x5A0))
    assert cv.detect_cell_channel([str(log)]) == CHANNEL


def test_partial_capture_needs_the_partial_flag(tmp_path):
    """A capture covering only part of the pack yields nothing by default."""
    log = tmp_path / "partial.log"
    half = set(range(49, N_CELLS + 1))
    _write_log(log, half)

    assert cv.collect_sweeps([str(log)], progress=False) == []

    sweeps = cv.collect_sweeps([str(log)], progress=False, partial=True)
    assert len(sweeps) == 1
    assert len(sweeps[0]["volts"]) == len(half)
    assert set(sweeps[0]["volts"]) == half


def test_partial_readouts_describe_only_captured_cells(tmp_path):
    log = tmp_path / "partial.log"
    half = set(range(49, N_CELLS + 1))
    _write_log(log, half)
    sweep = cv.collect_sweeps([str(log)], progress=False, partial=True)[0]
    assert sweep["max"] == max(_volts_for(c) for c in half)
    assert sweep["min"] == min(_volts_for(c) for c in half)


# ----------------------------------------------------------------- render
def test_render_png(tmp_path):
    pytest.importorskip("PIL")
    log = tmp_path / "full.log"
    _write_log(log, set(range(1, N_CELLS + 1)))
    sweep = cv.collect_sweeps([str(log)], progress=False)[0]
    out = tmp_path / "grid.png"
    cv.render_png(sweep["volts"], "corrected", str(out), scale=1)
    assert out.exists() and out.stat().st_size > 0

    from PIL import Image
    w, h = Image.open(out).size
    assert (w, h) == layout.WINDOW["31kWh"]


def test_render_marks_unseen_cells_with_placeholder(tmp_path):
    """Cells never captured keep the original's NN_k text, not a voltage."""
    assert cv._cell_text(49, {}) == "12_0"        # 0x00C slot 0
    assert cv._cell_text(49, {49: 3.325}) == "3.325"


# -------------------------------------------------------------------- cli
def test_cli_render_end_to_end(tmp_path):
    pytest.importorskip("PIL")
    log = tmp_path / "full.log"
    _write_log(log, set(range(1, N_CELLS + 1)))
    out = tmp_path / "cli.png"
    r = subprocess.run(
        [sys.executable, os.path.join(VIEWER_DIR, "cell_viewer.py"),
         "--log", str(log), "--render", str(out)],
        capture_output=True, text=True, timeout=120)
    assert r.returncode == 0, r.stdout + r.stderr
    assert out.exists()


# ------------------------------------------------- the fake cell broadcaster
# playback/coda_fake_cells.py is the transmit side used to exercise --live
# against real hardware. It builds frames independently of layout.py, so these
# guard against the two drifting apart.
def test_fake_cells_round_trips_through_the_decoder():
    mv = fake.pack_profile(low_cell=56)
    state = cv.PackState()

    class _F:
        pass

    published = False
    for can_id, data in fake.frames_for(mv):
        f = _F()
        f.timestamp, f.arbitration_id, f.data = 0.0, can_id, data
        published = state.update(f) or published

    assert published, "no sweep published; 0x019 should publish"
    assert state.complete
    for cell, millivolts in mv.items():
        assert state.volts[cell] == pytest.approx(millivolts / 1000.0)


def test_fake_cells_low_cell_becomes_the_minimum():
    mv = fake.pack_profile(low_cell=56)
    assert min(mv, key=mv.get) == 56
    assert mv[56] == 3243


def test_fake_cells_covers_every_cell_exactly_once():
    frames = fake.frames_for(fake.pack_profile())
    assert len(frames) == 26
    seen = [cell
            for can_id, data in frames
            for cell, _volts in layout.decode_frame(can_id, data)]
    assert sorted(seen) == list(range(1, N_CELLS + 1))


# ------------------------------------------------------------ replay source
def test_sweep_source_loops_by_default(tmp_path):
    """A short capture should keep playing rather than stopping."""
    log = tmp_path / "full.log"
    _write_log(log, set(range(1, N_CELLS + 1)))
    sweeps = cv.collect_sweeps([str(log)], progress=False)
    assert len(sweeps) == 1

    src = cv.sweep_source(sweeps)
    pulled = [next(src) for _ in range(5)]      # would StopIteration if not looping
    assert len(pulled) == 5
    assert all(p[0] == pulled[0][0] for p in pulled)
    src.close()


def test_sweep_source_no_loop_stops(tmp_path):
    log = tmp_path / "full.log"
    _write_log(log, set(range(1, N_CELLS + 1)))
    sweeps = cv.collect_sweeps([str(log)], progress=False)
    assert len(list(cv.sweep_source(sweeps, loop=False))) == len(sweeps)


# ------------------------------------------------------------ demo fallback
def test_demo_falls_back_when_no_even_column_extreme(tmp_path):
    """--at demo prefers an even-column extreme but must not refuse without one."""
    volts = {n: 3.300 for n in range(1, N_CELLS + 1)}
    pos = layout.cell_positions("corrected")
    odd = [n for n in volts if pos[n][1] % 2 == 1]
    volts[odd[0]] = 3.400                       # both extremes in odd columns
    volts[odd[1]] = 3.200
    sweeps = [{"t": 0.0, "min": 3.200, "max": 3.400, "volts": volts}]

    idx, sweep, row = cv.pick_sweep(sweeps, "demo", "corrected")
    assert idx == 0
    assert row["spread"] == pytest.approx(0.200)
