"""Coda pack cell layout -- geometry and cell-position mapping.

Two mappings are provided:

  "app"        -- exactly what Cell_Voltage_Display.exe draws.  Every module is
                  laid out with the cell number DESCENDING as the screen row
                  increases.  Faithful to the original, and wrong for 44 cells.

  "corrected"  -- the physical layout recovered from the pack's high-voltage
                  bussing drawing.  Identical to "app" except that
                  EVEN-numbered columns are mirrored top-to-bottom
                  within their own module.

Only the within-module cell ORDER differs.  Module membership, column position
and colour are the same in both -- no cell ever changes module or column.

Graphics constants below are reproduced from the original tool
(Cell_Voltage_Display.exe), so the rebuild is visually identical to it.
"""

# ---------------------------------------------------------------- original look
TITLE = "Complete Cell Voltage Display"
SCOPE_TITLE = "Intergrated Cell Voltage Display"   # sic -- original's typo
WINDOW = {"31kWh": (1250, 400), "36kWh": (1110, 450)}
WINDOW_POS = (100, 10)
SCOPE_WINDOW = (1100, 200)
SCOPE_POS = (100, 500)

FONT = ("courier new", 14, "bold")
CELL_WIDTH_CHARS = 12
RELIEF_CELL = "sunken"
RELIEF_HEADER = "raised"

# fg_colors / bg_colors, indexed by quadrant 0..3
FG_COLORS = ["white", "white", "black", "black"]
BG_COLORS = ["black", "blue", "white", "green"]
MAX_COLORS = {"fg": "black", "bg": "red"}
MIN_COLORS = {"fg": "white", "bg": "purple"}
MAXMIN_ROWS = (14, 15)          # tk grid rows of the Maximum / Minimum readouts

# scope-window scaling, reproduced from the original
SCOPE_SAMPLES = 183
SCOPE_V_MIN, SCOPE_V_SPAN = 3.0, 0.8

# ------------------------------------------------------------------- pack model
# (column, half) -> (first_cell, last_cell).  The string runs 1..52 across the
# top half left-to-right, through the HVD between 52 and 53, then 53..104 back
# across the bottom half right-to-left.
CELL_RANGE_31 = {
    (1, "top"): (1, 5),    (2, "top"): (6, 10),   (3, "top"): (11, 16),
    (4, "top"): (17, 22),  (5, "top"): (23, 28),  (6, "top"): (29, 34),
    (7, "top"): (35, 40),  (8, "top"): (41, 46),  (9, "top"): (47, 52),
    (9, "bot"): (53, 58),  (8, "bot"): (59, 64),  (7, "bot"): (65, 70),
    (6, "bot"): (71, 76),  (5, "bot"): (77, 82),  (4, "bot"): (83, 88),
    (3, "bot"): (89, 94),  (2, "bot"): (95, 99),  (1, "bot"): (100, 104),
}
NROW_31, NCOL_31 = 12, 9
HVD_BETWEEN = (52, 53)          # the manual service disconnect
PACK_TERMINALS = {"cell1": (6, 1), "cell104": (7, 1)}   # (screen_row, screen_col)


def module_rows(col, half):
    """Screen rows this module occupies, ordered top of screen -> bottom."""
    if half == "top":
        return [2, 3, 4, 5, 6] if col <= 2 else [1, 2, 3, 4, 5, 6]
    return [7, 8, 9, 10, 11] if col <= 2 else [7, 8, 9, 10, 11, 12]


def quadrant(col, half):
    """Index into FG_COLORS / BG_COLORS."""
    if half == "top":
        return 0 if col <= 5 else 1      # black / blue
    return 3 if col <= 5 else 2          # green / white


def build(mapping="corrected"):
    """-> dict (screen_row, screen_col) -> (cell_number, quadrant_index)."""
    if mapping not in ("app", "corrected"):
        raise ValueError(f"mapping must be 'app' or 'corrected', got {mapping!r}")
    grid = {}
    for (col, half), (lo, hi) in CELL_RANGE_31.items():
        rows = module_rows(col, half)
        cells = list(range(hi, lo - 1, -1))          # app: descending down-screen
        if mapping == "corrected" and col % 2 == 0:  # even columns are mirrored
            cells.reverse()
        assert len(rows) == len(cells), (col, half)
        q = quadrant(col, half)
        for r, n in zip(rows, cells):
            grid[(r, col)] = (n, q)
    assert len(grid) == 104
    return grid


def cell_positions(mapping="corrected"):
    """-> dict cell_number -> (screen_row, screen_col)."""
    return {n: rc for rc, (n, _q) in build(mapping).items()}


def module_of(cell):
    """-> (column, half, size, position_from_midline)."""
    for (col, half), (lo, hi) in CELL_RANGE_31.items():
        if lo <= cell <= hi:
            rows = module_rows(col, half)
            pos = cell_positions("corrected")[cell]
            seq = rows if half == "bot" else rows[::-1]   # mid-line first
            return col, half, len(rows), seq.index(pos[0]) + 1
    raise ValueError(cell)


# ------------------------------------------------------------------ CAN framing
CELL_FRAME_IDS = range(0x000, 0x01A)     # 26 frames on C CAN


def cell_from_frame(can_id, byte_index):
    """CAN id + 16-bit slot (0..3) -> 1-based DBC cell number."""
    return 4 * can_id + byte_index + 1


def decode_frame(can_id, data):
    """-> list of (cell_number, volts) for one C CAN cell frame."""
    if can_id not in CELL_FRAME_IDS or len(data) < 8:
        return []
    out = []
    for k in range(4):
        mv = (data[2 * k] << 8) | data[2 * k + 1]
        out.append((cell_from_frame(can_id, k), mv / 1000.0))
    return out
