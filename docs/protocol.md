# Coda Diagnostics — Protocol Reference

Technical notes too detailed for inline comments. Covers the non-standard
encoding layers that sit above the ISO-TP transport.

---

## 1. UDS service variants per module

| Module        | Request ID | Response ID | UDS service | Notes                        |
|---------------|-----------|-------------|-------------|------------------------------|
| BMS           | 0x722     | 0x72A       | 0x19 02 0C  | 24-bit DTC entries           |
| DLCM          | 0x7E0     | 0x7E8       | 0x19 02 0C  | 16-bit DTC, DLCM encoding    |
| Gateway       | 0x710     | 0x718       | 0x19 02 0C  | 16-bit, chassis16 decode     |
| HVAC          | 0x750     | 0x758       | 0x19 02 0C  | 16-bit, chassis16 decode     |
| ACCompressor  | 0x7C7     | 0x7CF       | 0x19 02 0C  | 16-bit, DLCM decode          |
| PowerSteering | 0x724     | 0x734       | 0x19 02 0C  | 16-bit EPS encoding          |
| ABS           | 0x784     | 0x785       | 0x18 00 FF  | UDS-18 (not 19); 16-bit DTC  |
| Airbag        | 0x7C4     | 0x7CC       | 0x18 02 FF  | UDS-18; sends 0x78 pending   |

**UDS 0x19 (ReadDTCInformation)** is the standard service used by most modules.
`0x19 02 0C` = subfn 0x02 (reportDTCByStatusMask), mask 0x0C (confirmed + pending).
Positive response: `0x59 02 <mask> <DTC bytes...>`.

**UDS 0x18** is a Continental/Hyundai variant used by ABS and Airbag.
`0x18 <subfn> <mask> 0x00`. Positive response byte is `0x58` (not `0x59`).

---

## 2. Per-module DTC encoding

### 2.1 BMS — 24-bit entries, `parse_bms` / `dec_bms`

Response layout after ISO-TP reassembly:

```
59 02 <mask> [b0 b1 b2 status] ...
```

Each 4-byte entry: `b0 b1 b2` form a 24-bit value; `dec_bms` uses only the
upper two bytes (`b0`, `b1`) to reconstruct the SAE code:

```python
b0 = (d >> 16) & 0xFF
b1 = (d >> 8) & 0xFF
# P1B1D → b0=0x1B, b1=0x1D → "P1B1D"
```

The third byte (`b2`, which becomes the low byte of `d`) is an artifact of how
the 3-byte read works; it carries part of the real payload but is ignored in
the decode because the BMS DTC space only uses the first two nibble pairs.

### 2.2 DLCM / Gateway / HVAC / ACCompressor — `parse_dlcm` / `dec_dlcm`

Response layout:

```
59 02 [b0 b1 b2 status] ...
```

Note the **absence of a mask byte** between `59 02` and the first DTC entry,
which is why `parse_dlcm` starts at `i=2` (not `i=3` as in `parse_bms`):

```python
i = 2
dtc = (raw[i] << 16) | (raw[i+1] << 8) | raw[i+2]
```

`dec_dlcm` then isolates the lower 16 bits and strips the proprietary bit-14
status flag before decoding the system/severity prefix:

```python
x = d & 0xFFFF        # discard the upper byte read in b0
x2 = x & ~0x4000      # strip bit 14 (Continental status flag)
s = (x2 >> 12) & 0xF  # top nibble → system prefix
```

**Why `d & 0xFFFF`**: `parse_dlcm` reads 3 bytes into a 24-bit int; the DLCM
DTC is encoded in the middle and lower bytes only. The upper byte (`b0`) is a
mask/overhead byte and is discarded here.

**Why `& ~0x4000`**: Continental DLCM sets bit 14 of the 16-bit DTC word to
indicate a specific fault category. This bit must be masked out before the
system-prefix nibble (`s`) is decoded, or the wrong service category is returned.

Wire example: `0x40 0x81 0x5A 0x8C`
- `d = 0x40815A`
- `x = 0x815A`
- `x2 = 0x015A` (bit 14 stripped)
- `s = 0x0` → `"P0"` prefix → result `"P015A"`

### 2.3 ABS — `parse_abs` / `dec_abs`, UDS-18 response

Response layout:

```
58 <status_mask> [hi lo status] ...
```

Continental ABS encodes status bits in the high byte of each DTC pair.
`parse_abs` strips them before passing to `dec_abs`:

```python
hi &= 0x3F   # clears bits 7 and 6 (status flags 0x80, 0x40)
```

`dec_abs` then maps specific `(hi, lo)` patterns to SAE code strings.
Notable quirk: the 0x62xx family maps to C22xx (ABS nibble-swap):

```python
if hi == 0x02:
    sw = ((lo & 0x0F) << 4) | ((lo & 0xF0) >> 4)
    return f"C22{sw:02X}"
```

### 2.4 Airbag — `parse_airbag` / `dec_chassis16`, UDS-18 + 0x78 pending

The Airbag ECU always sends a UDS response-pending frame first:

```
03 7F 18 78 00 00 00 00   (NRC 0x78 = requestCorrectlyReceivedResponsePending)
```

followed by the real response. The ISO-TP transport (`recv_isotp_stack`) detects
and discards these pending frames transparently:

```python
if len(payload) >= 3 and payload[0] == 0x7F and payload[2] == 0x78:
    continue
```

Airbag DTC bytes have `0x80` OR'd into the high byte (distinct from ABS's `0x40`).
`parse_airbag` does **not** strip these — `dec_chassis16` handles the full encoded
value correctly because `(x >> 12) & 0xF` lands in the `8..B` range, which maps
to the `"B"` system prefix.

### 2.5 HVAC — `parse_dlcm` / `dec_chassis16`

HVAC uses the same UDS-19 response format as DLCM but is decoded with
`dec_chassis16` (not `dec_dlcm`), because HVAC uses `0x80`-prefixed high bytes
(same as Airbag) rather than the DLCM bit-14 pattern.

### 2.6 PowerSteering (EPS) — `parse_eps` / `dec_eps`

EPS responses are 16-bit DTC entries (2 bytes each, no third byte):

```python
i = 2
dtc = (raw[i] << 8) | raw[i+1]
st = raw[i+2]
i += 3
```

`dec_eps` maps specific top-nibble values to C-system codes.

---

## 3. ISO-TP transport layer

The reader uses `python-can-isotp` (`import isotp`) for all transport.
A fresh `isotp.CanStack` is created per request (each module has a unique
`(txid, rxid)` pair; `CanStack` cannot be readdressed after construction).
The shared `can.Bus` object stays open across all module queries.

Flow control (FC) parameters sent to the ECU:

| Parameter   | Value | Meaning                                      |
|-------------|-------|----------------------------------------------|
| `stmin`     | 0     | No inter-frame gap required                  |
| `blocksize` | 0     | No intermediate FC needed (single block)     |
| `tx_padding`| 0     | Pad outgoing frames to 8 bytes with `0x00`   |

These match the previous hand-rolled behavior (`30 00 00` FC frame + zero-padded SF).

---

## 4. Regression test setup

### Prerequisites

```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
```

Verify with `ip link show vcan0`.

### Running

```bash
pip install python-can python-can-isotp pytest
pytest test_regression.py -v
```

**Do not** use `-n` (xdist parallel) — all tests share `vcan0` and will
corrupt each other if run concurrently.

### Test coverage

| Test                | Fake-car args                       | Asserts         | Multi-frame? |
|---------------------|-------------------------------------|-----------------|--------------|
| `test_bms_dtc`      | `--bms-dtc P1B1D`                   | `P1B1D` in out  | No (7 bytes) |
| `test_abs_dtc`      | `--abs-dtc C102B`                   | `C102B` in out  | No (5 bytes) |
| `test_airbag_dtc`   | `--airbag-dtc B2505`                | `B2505` in out  | No (5 bytes) |
| `test_hvac_dtc`     | `--hvac-dtc B111D`                  | `B111D` in out  | No (6 bytes) |
| `test_multi_module` | all four + `--bms-dtc P1B1D P1B1E`  | all four in out | Yes (BMS→11) |

`test_multi_module` uses two BMS DTCs to push the BMS response to 11 bytes,
which forces the fake car to send a multi-frame (FF + CF) and exercises the
full ISO-TP reassembly path in the reader.
