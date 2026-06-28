"""
Pure decoder unit tests for read_dtcs_coda.py.

Unlike test_regression.py these need no vcan/socketcan and run on any platform:
    pytest test_decode.py -v

The payloads are the real ISO-TP-reassembled responses captured from a Coda
full-DTC read (projects/coda/logs/coda_srs_dtc_read.log). They lock in the
dec_sae16 fix: Airbag/Gateway U-codes (formerly printed "?xxxx") and DLCM
C0044 (formerly mis-decoded "P0044").
"""
import importlib

import pytest

reader = importlib.import_module("read_dtcs_coda")


def _hex(s):
    return bytes.fromhex(s.replace(" ", ""))


@pytest.mark.parametrize("value,expected", [
    (0x9102, "B1102"), (0xD110, "U1110"), (0xC110, "U0110"),
    (0x9513, "B1513"), (0x9382, "B1382"), (0x4044, "C0044"),
    (0x6100, "C2100"), (0x3115, "P3115"), (0xD074, "U1074"),
    (0xC037, "U0037"), (0xC155, "U0155"),
    (0xA505, "B2505"), (0x911D, "B111D"),
])
def test_dec_sae16(value, expected):
    assert reader.dec_sae16(value) == expected


def test_airbag_real_payload():
    raw = _hex("58 05 91 02 20 D1 10 20 C1 10 20 95 13 20 93 82 20")
    _, dtcs = reader.parse_airbag(raw)
    assert [reader.dec_sae16(d) for d, _ in dtcs] == \
        ["B1102", "U1110", "U0110", "B1513", "B1382"]


def test_gateway_real_payload():
    raw = _hex("59 02 0C 91 02 11 08 D0 74 88 08 C0 37 00 08")
    _, dtcs = reader.parse_dlcm(raw)
    assert [reader.dec_sae16(d) for d, _ in dtcs] == ["B1102", "U1074", "U0037"]


def test_dlcm_real_payload():
    raw = _hex("59 02 DF 40 44 13 08 C1 10 08 08 61 00 93 08 31 15 04 08 C1 10 08 08")
    _, dtcs = reader.parse_dlcm(raw)
    assert [reader.dec_sae16(d) for d, _ in dtcs] == \
        ["C0044", "U0110", "C2100", "P3115", "U0110"]
