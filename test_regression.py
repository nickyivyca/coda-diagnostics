"""
Regression tests for read_dtcs_coda.py (isotp-refactor branch).

Prerequisites (Linux/WSL only):
    sudo modprobe vcan
    sudo ip link add dev vcan0 type vcan
    sudo ip link set up vcan0

Run with:
    pytest test_regression.py -v

Do NOT run with -n (parallel); both scripts share vcan0.
"""
import subprocess
import sys
import time
import os

import pytest

# ---------------------------------------------------------------------------
# Platform guard — these tests require Linux vcan support
# ---------------------------------------------------------------------------
pytestmark = pytest.mark.skipif(
    sys.platform != "linux",
    reason="vcan tests require Linux",
)

FAKE_CAR = os.path.join(os.path.dirname(__file__), "playback", "coda_fake_car.py")
READER = os.path.join(os.path.dirname(__file__), "read_dtcs_coda.py")
IFACE = "socketcan"
CHAN = "vcan0"
READER_TIMEOUT = 30  # seconds — 8 modules × ~5 s timeout each in the worst case


def _run_reader():
    """Run the DTC reader against vcan0 and return its stdout."""
    result = subprocess.run(
        [sys.executable, READER, "--interface", IFACE, "--channel", CHAN],
        capture_output=True,
        text=True,
        timeout=READER_TIMEOUT,
    )
    return result.stdout


def _start_fake_car(*extra_args):
    """Start the fake car simulator as a background process."""
    cmd = [
        sys.executable, FAKE_CAR,
        "--interface", IFACE,
        "--channel", CHAN,
    ] + list(extra_args)
    return subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


# ---------------------------------------------------------------------------
# Individual module tests
# ---------------------------------------------------------------------------

def test_bms_dtc():
    car = _start_fake_car("--bms-dtc", "P1B1D")
    try:
        time.sleep(1.5)
        out = _run_reader()
    finally:
        car.terminate()
        car.wait()
    assert "P1B1D" in out, f"P1B1D not found in reader output:\n{out}"


def test_abs_dtc():
    car = _start_fake_car("--abs-dtc", "C102B")
    try:
        time.sleep(1.5)
        out = _run_reader()
    finally:
        car.terminate()
        car.wait()
    assert "C102B" in out, f"C102B not found in reader output:\n{out}"


def test_airbag_dtc():
    car = _start_fake_car("--airbag-dtc", "B2505")
    try:
        time.sleep(1.5)
        out = _run_reader()
    finally:
        car.terminate()
        car.wait()
    assert "B2505" in out, f"B2505 not found in reader output:\n{out}"


def test_hvac_dtc():
    car = _start_fake_car("--hvac-dtc", "B111D")
    try:
        time.sleep(1.5)
        out = _run_reader()
    finally:
        car.terminate()
        car.wait()
    assert "B111D" in out, f"B111D not found in reader output:\n{out}"


def test_multi_module():
    """
    All four modules active simultaneously.
    Two BMS DTCs push the BMS response over 7 bytes, exercising the multi-frame
    FF/FC/CF path through the isotp stack.
    """
    car = _start_fake_car(
        "--bms-dtc", "P1B1D", "P1B1E",
        "--abs-dtc", "C102B",
        "--airbag-dtc", "B2505",
        "--hvac-dtc", "B111D",
    )
    try:
        time.sleep(1.5)
        out = _run_reader()
    finally:
        car.terminate()
        car.wait()
    assert "P1B1D" in out, f"P1B1D not found in multi-module output:\n{out}"
    assert "C102B" in out, f"C102B not found in multi-module output:\n{out}"
    assert "B2505" in out, f"B2505 not found in multi-module output:\n{out}"
    assert "B111D" in out, f"B111D not found in multi-module output:\n{out}"
