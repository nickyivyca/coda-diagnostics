# Playback

Scripts that put CAN traffic on a bus so the tools in this repo can be tested
without a vehicle.

| Script | What it does |
|---|---|
| `coda_fake_car.py` | Simulates a Coda answering UDS DTC requests, so `read_dtcs_coda.py` and `clear_dtcs_coda.py` can be exercised end to end. This is what `../test_regression.py` drives over vcan. It can also replay raw module responses captured from a real car. Incomplete, but covers ABS, AC compressor, airbag, BMS and HVAC. |
| `coda_fake_cells.py` | Broadcasts synthetic BMS cell voltages on C CAN (26 frames, IDs 0x000-0x019, 104 cells) for testing `../cell_viewer/`. The viewer's `--live` path opens a real CAN channel, so no automated test can cover it; run this on one dongle and the viewer on another wired back to back. `--dry-run` prints the frames without opening hardware. |

`coda_fake_car.py` is also used to replay logs back into the Omitec scanner to
recover a code description when it is not already in the DTC list CSV.

## Two dongles, back to back

```bash
python playback/coda_fake_cells.py --interface pcan --channel PCAN_USBBUS1 --low-cell 56
python cell_viewer/cell_viewer.py --live --interface kvaser --channel 0 --gui
```

`--low-cell` drives one cell well below the rest so the Minimum readout and its
position on the grid are obvious, rather than 104 near-identical numbers.

## One machine, no dongles (Linux)

Both scripts work over a virtual CAN pair:

```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0
```

Without root, an unprivileged network namespace also works, and keeps the
interface isolated from the rest of the machine:

```bash
unshare --user --map-root-user --net -- bash -c \
  'ip link add dev vcan0 type vcan && ip link set up vcan0 && <your command>'
```
