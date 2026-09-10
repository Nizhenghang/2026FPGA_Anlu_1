#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Cycle-accurate model of marquee_overlay.v (the scrolling slogan banner).

No Verilog simulator on this machine, so this mirrors the RTL register for
register and drives it with the real color_bar 640x480 raster. Every geometry
constant, bit slice, register width and colour is parsed out of
marquee_overlay.v -- nothing is restated here, so the model cannot silently
drift from the RTL. If the RTL changes shape the parser stops with "the model is
stale, fix the parser" instead of quietly simulating something else.

The glyph bitmaps come from marquee_font.vh (parsed with
gen_marquee_font.parse_vh), i.e. from exactly the table the RTL will synthesise,
not from a fresh render. Comparing that table against a fresh render is
check_marquee_transcription.py's job.

Passes (Gate 2 of the plan):

  A  Raster tracker. x_pos is the active pixel index 0..639 and y_pos the line
     index 0..479 on every cycle of a full frame; de_d is I_de delayed one
     clock; x_pos is 0 in blanking; exactly one frame_wrap per frame, on the
     first blanking cycle after the last line; and the state at the first active
     cycle of a line is always (x_pos=0, y_pos=line, de_d=0) -- that last fact
     is what legitimises the presets Pass D uses. Run under two blanking
     lengths to show the tracker only depends on the active counts.
  B  The borrow-wrap addressing trick, exhaustively: all 640 x 1120 = 716800
     (x_pos, marq_pos) pairs. in_region computed on the 11 bit wire must equal
     the unbounded-integer predicate 640 <= x + pos < 1120, and cell_idx/col
     must be the exact quotient and remainder of the 32 px pitch division.
  C  Geometry. The band is centred on y=240, its 32 rows are 1+3+24+3+1, it
     touches neither the OSD panel nor the spectrum panel, the 1 px edges land
     on the first and last band row, the parsed localparams and colours agree
     with the generator, and the six padding rows alias to glyph rows 24..31 --
     above the box, therefore always zero. That last fact is measured, not
     assumed: the band is rendered with and without in_text_rows and must come
     out identical, which is why the gate is free insurance today and why the
     check fails loudly if CELL ever grows into the padding.
  D  Whole-band render compared pixel for pixel against
     gen_marquee_font.band_pixel -- the golden reference model -- at eleven
     marq_pos values including both ends of the travel, on three backgrounds;
     plus the scroll cadence (1 px every SCROLL_FRAME_DIV frames, wrapping at
     TRAVEL), the SW4 mask, and the dim arithmetic.
  E  Negative controls, each of which must turn its assertion red: E1 the
     cell_idx slice narrowed to u[9:4], E2 row without the ROW_LSB
     subtraction, E3 glyph_bits[gcol] instead of [23-gcol], E4 marq_pos
     advancing every frame, E5 I_en dropped from in_band.

Run from anywhere:  python tools/sim_marquee.py
"""
import os
import re
import sys

TOOLS = os.path.dirname(os.path.abspath(__file__))
if TOOLS not in sys.path:
    sys.path.insert(0, TOOLS)

import gen_marquee_font as gen   # golden band_pixel + the one .vh parser

HDL = os.path.join(os.path.dirname(TOOLS), "src", "user_source", "hdl_source")
RTL = os.path.join(HDL, "marquee_overlay.v")
VH = gen.VH_PATH
OSD_RTL = os.path.join(HDL, "osd_overlay.v")
VIS_RTL = os.path.join(HDL, "audio_visualizer.v")

# color_bar.v's `VIDEO_640_480 block: H_ACTIVE 640 + FP 16 + SYNC 96 + BP 48,
# V_ACTIVE 480 + FP 10 + SYNC 2 + BP 33. de is high for exactly 640 clocks per
# line and there are 480 such lines per frame. The marquee RTL does not contain
# these numbers, and Pass A proves the tracker does not care about the blanking
# lengths by running the same assertions with a 1-cycle blank.
H_TOTAL = 800
V_TOTAL = 525

FAILURES = []
CHECKS = [0]


def check(cond, label, detail=""):
    CHECKS[0] += 1
    if cond:
        print("    ok    %s" % label)
        return True
    FAILURES.append("%s%s" % (label, (" -- " + detail) if detail else ""))
    print("    FAIL  %s%s" % (label, (" -- " + detail) if detail else ""))
    return False


def expect_fail(cond, label, detail=""):
    """Negative control: passes when cond is False."""
    CHECKS[0] += 1
    if not cond:
        print("    ok    control bites: %s" % label)
        return True
    FAILURES.append("control did NOT bite: %s" % label)
    print("    FAIL  control did NOT bite: %s%s"
          % (label, (" -- " + detail) if detail else ""))
    return False


def read(path):
    with open(path, "r", encoding="utf-8") as fh:
        return fh.read()


def stale(why):
    raise SystemExit("sim_marquee: %s -- the model is stale, fix the parser" % why)


# ---------------------------------------------------------------------------
# RTL parsing. Everything the model uses is lifted out of the .v file.
# ---------------------------------------------------------------------------
_LITERAL_RE = re.compile(r"(\d+)\s*'\s*([dDhHbBoO])\s*([0-9a-fA-F_]+)")
_BASES = {"d": 10, "h": 16, "b": 2, "o": 8}


def _literal_to_py(m):
    return str(int(m.group(3).replace("_", ""), _BASES[m.group(2).lower()]))


def strip_comments(code):
    code = re.sub(r"/\*.*?\*/", "", code, flags=re.S)
    return re.sub(r"//[^\n]*", "", code)


def eval_verilog_int(expr, known):
    """Evaluate a constant Verilog expression with the names resolved so far."""
    e = _LITERAL_RE.sub(_literal_to_py, expr.strip())
    if not re.match(r"^[0-9A-Za-z_ \+\-\*/%\(\)]+$", e):
        raise ValueError("cannot evaluate %r" % expr)
    e = e.replace("/", "//")           # Verilog integer division truncates
    return int(eval(e, {"__builtins__": {}}, dict(known)))


_PARAM_RE = re.compile(
    r"\bparameter\s+(?:\[\s*\d+\s*:\s*\d+\s*\]\s*)?(\w+)\s*=\s*([^,\)]+)")
_LOCALPARAM_RE = re.compile(
    r"\blocalparam\s+(?:\[\s*\d+\s*:\s*\d+\s*\]\s*)?(\w+)\s*=\s*([^;]+);")
_WIDTH_RE = re.compile(r"\b(?:reg|wire)\s+\[\s*(\d+)\s*:\s*0\s*\]\s*(\w+)\s*;")

REQUIRED_PARAMS = (
    "H_ACTIVE", "V_ACTIVE", "CELL", "PITCH", "GUTTER", "N_CELLS", "TEXT_W",
    "TRAVEL", "BAND_H", "BAND_Y", "BAND_Y_LAST", "BAND_TEXT_Y",
    "BAND_TEXT_Y_LAST", "ROW_LSB", "SCROLL_FRAME_DIV", "DIM_SHIFT",
    "FRAME_DIV_LAST", "H_ACTIVE_W", "V_ACTIVE_W", "BAND_Y_W", "BAND_Y_LAST_W",
    "BAND_TEXT_Y_W", "BAND_TEXT_Y_LAST_W", "H_ACTIVE_P", "TEXT_W_P",
    "TRAVEL_LAST_P", "GUTTER_W", "GUTTER_LAST_W", "ROW_LSB_W",
)

REQUIRED_WIDTHS = (
    "x_pos", "y_pos", "marq_pos", "frame_div", "s", "u", "cell_idx", "col",
    "gcol", "row", "glyph_bits",
)


def parse_params(code, path):
    header = re.search(r"module\s+\w+\s*#\s*\((.*?)\)\s*\(", code, flags=re.S)
    found = [(m.group(1), m.group(2).strip())
             for m in _PARAM_RE.finditer(header.group(1) if header else "")]
    found += [(m.group(1), m.group(2).strip()) for m in _LOCALPARAM_RE.finditer(code)]

    cfg = {}
    pending = list(found)
    for _ in range(len(pending) + 1):
        if not pending:
            break
        unresolved = []
        for name, expr in pending:
            try:
                cfg[name] = eval_verilog_int(expr, cfg)
            except (ValueError, NameError, KeyError, TypeError, SyntaxError):
                unresolved.append((name, expr))
        if len(unresolved) == len(pending):
            stale("cannot resolve %s out of %s"
                  % ([n for n, _ in unresolved], path))
        pending = unresolved
    missing = [n for n in REQUIRED_PARAMS if n not in cfg]
    if missing:
        stale("missing localparams %s in %s" % (missing, path))
    return cfg


def parse_widths(code, path):
    widths = {m.group(2): int(m.group(1)) + 1 for m in _WIDTH_RE.finditer(code)}
    missing = [n for n in REQUIRED_WIDTHS if n not in widths]
    if missing:
        stale("no width declared for %s in %s" % (missing, path))
    return widths


def assign_rhs(code, name):
    m = re.search(r"assign\s+" + re.escape(name) + r"\s*=\s*(.*?);", code, flags=re.S)
    if not m:
        stale("no `assign %s` in the RTL" % name)
    return re.sub(r"\s+", " ", m.group(1)).strip()


def match_or_stale(pattern, text, what):
    m = re.match(pattern, text)
    if not m:
        stale("%s: expected %s, RTL says `%s`" % (what, pattern, text))
    return m


def parse_ops(code, cfg):
    """The combinational heart of the module, as structured data."""
    ops = {}

    txt = assign_rhs(code, "s")
    match_or_stale(r"\{\s*1'b0\s*,\s*x_pos\s*\}\s*\+\s*marq_pos", txt,
                   "s must stay the zero-extended x_pos + marq_pos")
    txt = assign_rhs(code, "u")
    ops["u_sub"] = cfg[match_or_stale(r"s\s*-\s*(\w+)", txt, "u = s - <const>")[1]]
    txt = assign_rhs(code, "in_region")
    ops["region_lt"] = cfg[match_or_stale(r"\(?\s*u\s*<\s*(\w+)\s*\)?", txt,
                                          "in_region = u < <const>")[1]]

    txt = assign_rhs(code, "cell_idx")
    ops["cell_slice"] = tuple(int(g) for g in
                              match_or_stale(r"u\[\s*(\d+)\s*:\s*(\d+)\s*\]", txt,
                                             "cell_idx = u[hi:lo]").groups())
    txt = assign_rhs(code, "col")
    ops["col_slice"] = tuple(int(g) for g in
                             match_or_stale(r"u\[\s*(\d+)\s*:\s*(\d+)\s*\]", txt,
                                            "col = u[hi:lo]").groups())

    txt = assign_rhs(code, "col_in_glyph")
    lo_name, hi_name = match_or_stale(
        r"\(?\s*col\s*>=\s*(\w+)\s*\)?\s*&&\s*\(?\s*col\s*<=\s*(\w+)\s*\)?", txt,
        "col_in_glyph = (col >= <lo>) && (col <= <hi>)").groups()
    ops["col_lo"], ops["col_hi"] = cfg[lo_name], cfg[hi_name]

    txt = assign_rhs(code, "gcol")
    ops["gcol_sub"] = cfg[match_or_stale(r"col\s*-\s*(\w+)", txt,
                                        "gcol = col - <const>")[1]]

    txt = assign_rhs(code, "row")
    hi, lo, name = match_or_stale(
        r"y_pos\[\s*(\d+)\s*:\s*(\d+)\s*\]\s*-\s*(\w+)", txt,
        "row = y_pos[hi:lo] - <const>").groups()
    ops["row_slice"] = (int(hi), int(lo))
    ops["row_sub"] = cfg[name]

    txt = assign_rhs(code, "glyph_bits")
    cell_name, chi, clo, row_name = match_or_stale(
        r"marquee_glyph\(\s*(\w+)\[\s*(\d+)\s*:\s*(\d+)\s*\]\s*,\s*(\w+)\s*\)",
        txt, "glyph_bits = marquee_glyph(cell_idx[hi:lo], row)").groups()
    if cell_name != "cell_idx" or row_name != "row":
        stale("marquee_glyph is fed %s/%s, expected cell_idx/row"
              % (cell_name, row_name))
    ops["cell_arg_mask"] = (1 << (int(chi) - int(clo) + 1)) - 1

    txt = assign_rhs(code, "text_on")
    width, const, gcol_name = match_or_stale(
        r"in_text_rows\s*&&\s*in_region\s*&&\s*col_in_glyph\s*&&\s*"
        r"glyph_bits\[\s*(\d+)\s*'d\s*(\d+)\s*-\s*(\w+)\s*\]", txt,
        "text_on = in_text_rows && in_region && col_in_glyph "
        "&& glyph_bits[N'd<hi> - gcol]").groups()
    if gcol_name != "gcol":
        stale("the glyph bit index subtracts %s, expected gcol" % gcol_name)
    ops["index_width"] = int(width)
    ops["index_msb"] = int(const)

    txt = assign_rhs(code, "in_band")
    ops["en_gates_band"] = bool(re.match(r"I_en\s*&&\s*I_de\s*&&", txt))
    lo_name, hi_name = re.findall(
        r"\(?\s*y_pos\s*>=\s*(\w+)\s*\)?\s*&&\s*\(?\s*y_pos\s*<=\s*(\w+)\s*\)?",
        txt)[0]
    ops["band_lo"], ops["band_hi"] = cfg[lo_name], cfg[hi_name]

    txt = assign_rhs(code, "band_edge")
    a, b = match_or_stale(r"\(?\s*y_pos\s*==\s*(\w+)\s*\)?\s*\|\|\s*"
                          r"\(?\s*y_pos\s*==\s*(\w+)\s*\)?", txt,
                          "band_edge = (y_pos == a) || (y_pos == b)").groups()
    ops["edge_rows"] = (cfg[a], cfg[b])

    txt = assign_rhs(code, "in_text_rows")
    lo_name, hi_name = match_or_stale(
        r"\(?\s*y_pos\s*>=\s*(\w+)\s*\)?\s*&&\s*\(?\s*y_pos\s*<=\s*(\w+)\s*\)?",
        txt, "in_text_rows = (y_pos >= lo) && (y_pos <= hi)").groups()
    ops["text_rows"] = (cfg[lo_name], cfg[hi_name])

    txt = assign_rhs(code, "frame_wrap")
    ops["frame_wrap_expr"] = txt
    if not re.match(r"!I_de\s*&&\s*de_d\s*&&", txt):
        stale("frame_wrap is not `!I_de && de_d && (y_pos == last)`: %s" % txt)
    tail = txt.split("&&", 2)[2].strip()
    ops["frame_wrap_y"] = eval_verilog_int(
        match_or_stale(r"\(?\s*y_pos\s*==\s*(.+?)\s*\)?$", tail,
                       "frame_wrap's y_pos compare").group(1), cfg)

    txt = assign_rhs(code, "O_rgb")
    if not txt.startswith("!in_band ? I_rgb"):
        stale("O_rgb must pass I_rgb through outside the band: %s" % txt)
    text_hex, edge_hex = match_or_stale(
        r"!in_band \? I_rgb :\s*text_on \? (\d+'h[0-9A-Fa-f]+) :\s*"
        r"band_edge \? (\d+'h[0-9A-Fa-f]+) :\s*\{\s*dim_r\s*,\s*dim_g\s*,\s*dim_b\s*\}",
        txt, "the 4-way output mux").groups()
    ops["text_rgb"] = int(_LITERAL_RE.sub(_literal_to_py, text_hex))
    ops["edge_rgb"] = int(_LITERAL_RE.sub(_literal_to_py, edge_hex))

    for ch, bits in (("r", "[23:16]"), ("g", "[15:8]"), ("b", "[7:0]")):
        txt = assign_rhs(code, "dim_" + ch)
        match_or_stale(r"I_rgb" + re.escape(bits) + r"\s*>>\s*(\w+)", txt,
                       "dim_%s = I_rgb%s >> DIM_SHIFT" % (ch, bits))
        if cfg[txt.split(">>")[1].strip()] != cfg["DIM_SHIFT"]:
            stale("dim_%s shifts by something other than DIM_SHIFT" % ch)

    return ops


ALWAYS_FRAGMENTS = (
    "if (!de_d)",
    "x_pos <= 10'd1;",
    "else if (x_pos == H_ACTIVE_W - 10'd1)",
    "x_pos <= 10'd0;",
    "x_pos <= x_pos + 10'd1;",
    "y_pos <= 10'd0;",
    "y_pos <= y_pos + 10'd1;",
    "if (y_pos == V_ACTIVE_W - 10'd1)",
    "if (frame_div == FRAME_DIV_LAST)",
    "frame_div <= 6'd0;",
    "if (marq_pos == TRAVEL_LAST_P)",
    "marq_pos <= 11'd0;",
    "marq_pos <= marq_pos + 11'd1;",
    "frame_div <= frame_div + 6'd1;",
    "de_d <= I_de;",
)


def check_always_shape(code):
    """Lock the sequential block down: the model below transcribes these lines."""
    block = re.search(r"always\s*@\s*\(posedge.*?endmodule", code, flags=re.S)
    if not block:
        return ["no always block found"]
    body = re.sub(r"\s+", " ", block.group(0))
    missing = []
    for frag in ALWAYS_FRAGMENTS:
        if re.sub(r"\s+", " ", frag) not in body:
            missing.append(frag)
    return missing


def parse_rtl(path=RTL):
    raw = read(path)
    code = strip_comments(raw)
    cfg = parse_params(code, path)
    widths = parse_widths(code, path)
    ops = parse_ops(code, cfg)
    return cfg, widths, ops, raw


def load_font(path=VH):
    """Parse marquee_font.vh into a complete {cell: {row: bits}} table.

    The emitter folds all-zero rows into the `default` arm, so a parsed cell can
    be missing row keys. The RTL answers 24'h000000 for those, and filling them
    in here is what makes gen.band_pixel -- which indexes every row -- usable as
    the golden reference for Pass D.
    """
    parsed = gen.parse_vh(read(path))
    if sorted(parsed) != list(range(gen.N_CELLS)):
        stale("%s holds cells %s, expected 0..%d"
              % (path, sorted(parsed), gen.N_CELLS - 1))
    table = {}
    for cell in range(gen.N_CELLS):
        rows = parsed[cell]
        outside = sorted(r for r in rows if not 0 <= r < gen.CELL)
        if outside:
            stale("cell %d of %s indexes rows %s outside 0..%d"
                  % (cell, path, outside, gen.CELL - 1))
        table[cell] = {r: rows.get(r, 0) for r in range(gen.CELL)}
    return table


def parse_panel(path, what):
    """PANEL_Y / PANEL_H of the neighbouring overlays, read from their RTL."""
    code = strip_comments(read(path))
    cfg = {}
    pending = [(m.group(1), m.group(2).strip())
               for m in re.finditer(r"\blocalparam\s+(\w+)\s*=\s*([^;]+);", code)]
    for _ in range(len(pending) + 1):
        if not pending:
            break
        unresolved = []
        for name, expr in pending:
            try:
                cfg[name] = eval_verilog_int(expr, cfg)
            except (ValueError, NameError, KeyError, TypeError, SyntaxError):
                unresolved.append((name, expr))
        pending = unresolved
    for key in ("PANEL_Y", "PANEL_H"):
        if key not in cfg:
            stale("no %s in %s (%s bounds)" % (key, path, what))
    return cfg["PANEL_Y"], cfg["PANEL_Y"] + cfg["PANEL_H"] - 1


# ---------------------------------------------------------------------------
# The model
# ---------------------------------------------------------------------------
class Marquee(object):
    """marquee_overlay.v, register for register.

    comb() is the wire cloud as seen in the current cycle; step() applies the
    always block. The driver calls comb() BEFORE step(), which is what
    non-blocking assignment means: the outputs of a cycle are a function of the
    registers as they were at that clock edge.

    The attributes below are the only knobs the _Broken* controls touch, so a
    control can never accidentally rewrite the logic it is meant to perturb.
    """

    def __init__(self, cfg, widths, ops, font):
        self.cfg = cfg
        self.widths = widths
        self.ops = ops
        self.font = font
        self.mask_pos = (1 << widths["marq_pos"]) - 1
        self.mask_xy = (1 << widths["x_pos"]) - 1
        self.mask5 = (1 << widths["gcol"]) - 1
        self.mask_div = (1 << widths["frame_div"]) - 1
        # knobs
        self.cell_slice = ops["cell_slice"]
        self.col_slice = ops["col_slice"]
        self.row_sub = ops["row_sub"]
        self.gate_rows = True
        self.en_gates_band = ops["en_gates_band"]
        self.bit_order_reversed = False
        self.frame_div_last = cfg["FRAME_DIV_LAST"]
        self.reset()

    def reset(self):
        self.x_pos = 0
        self.y_pos = 0
        self.de_d = 0
        self.marq_pos = 0
        self.frame_div = 0

    def preset(self, x_pos, y_pos, de_d, marq_pos, frame_div=0):
        """Jump to a raster state Pass A has proven the tracker really visits."""
        self.x_pos = x_pos
        self.y_pos = y_pos
        self.de_d = de_d
        self.marq_pos = marq_pos
        self.frame_div = frame_div

    @staticmethod
    def _slice(value, hi, lo):
        return (value >> lo) & ((1 << (hi - lo + 1)) - 1)

    def addressing(self, x_pos, marq_pos):
        """s / u / in_region / cell / col / col_in_glyph / gcol, bit exact."""
        cfg = self.cfg
        s = (x_pos + marq_pos) & self.mask_pos
        u = (s - self.ops["u_sub"]) & self.mask_pos
        in_region = u < self.ops["region_lt"]
        cell = self._slice(u, *self.cell_slice)
        col = self._slice(u, *self.col_slice)
        col_in_glyph = self.ops["col_lo"] <= col <= self.ops["col_hi"]
        gcol = (col - self.ops["gcol_sub"]) & self.mask5
        return s, u, in_region, cell, col, col_in_glyph, gcol

    def glyph(self, cell, row):
        return self.font.get(cell & self.ops["cell_arg_mask"], {}).get(row, 0)

    def comb(self, de, rgb, en):
        """Returns (O_rgb, diagnostics). Pure function of registers + inputs."""
        cfg, ops = self.cfg, self.ops
        x, y = self.x_pos, self.y_pos

        in_band = bool(de) and ops["band_lo"] <= y <= ops["band_hi"]
        if self.en_gates_band:
            in_band = in_band and bool(en)
        band_edge = y == ops["edge_rows"][0] or y == ops["edge_rows"][1]
        in_text_rows = ops["text_rows"][0] <= y <= ops["text_rows"][1]
        frame_wrap = (not de) and bool(self.de_d) and y == ops["frame_wrap_y"]

        s, u, in_region, cell, col, col_in_glyph, gcol = self.addressing(
            x, self.marq_pos)
        row = (self._slice(y, *ops["row_slice"]) - self.row_sub) & self.mask5

        if self.bit_order_reversed:
            index = gcol & ((1 << ops["index_width"]) - 1)
        else:
            index = (ops["index_msb"] - gcol) & ((1 << ops["index_width"]) - 1)
        bits = self.glyph(cell, row)
        # An out-of-range bit select synthesises to 0; col_in_glyph masks it
        # anyway, but modelling it keeps the diagnostics honest.
        text_bit = (bits >> index) & 1 if index < self.widths["glyph_bits"] else 0
        text_on = in_region and col_in_glyph and bool(text_bit)
        if self.gate_rows:
            text_on = text_on and in_text_rows

        shift = cfg["DIM_SHIFT"]
        dim = (((rgb >> 16) & 0xFF) >> shift) << 16 \
            | (((rgb >> 8) & 0xFF) >> shift) << 8 \
            | ((rgb & 0xFF) >> shift)

        if not in_band:
            out = rgb
        elif text_on:
            out = ops["text_rgb"]
        elif band_edge:
            out = ops["edge_rgb"]
        else:
            out = dim

        diag = {"x": x, "y": y, "in_band": in_band, "band_edge": band_edge,
                "in_text_rows": in_text_rows, "frame_wrap": frame_wrap,
                "s": s, "u": u, "in_region": in_region, "cell": cell,
                "col": col, "col_in_glyph": col_in_glyph, "gcol": gcol,
                "row": row, "bits": bits, "index": index,
                "text_on": text_on, "dim": dim}
        return out, diag

    def step(self, de):
        """The always block. Every next value is computed before any commit."""
        cfg = self.cfg
        x, y, de_d = self.x_pos, self.y_pos, self.de_d
        pos, div = self.marq_pos, self.frame_div
        n_x, n_y, n_pos, n_div = x, y, pos, div
        n_de_d = 1 if de else 0

        if de:
            if not de_d:
                n_x = 1
            elif x == cfg["H_ACTIVE_W"] - 1:
                n_x = 0
            else:
                n_x = x + 1
        else:
            n_x = 0
            if de_d:
                if y == cfg["V_ACTIVE_W"] - 1:
                    n_y = 0
                    if div == self.frame_div_last:
                        n_div = 0
                        n_pos = 0 if pos == cfg["TRAVEL_LAST_P"] else pos + 1
                    else:
                        n_div = div + 1
                else:
                    n_y = y + 1

        self.x_pos = n_x & self.mask_xy
        self.y_pos = n_y & self.mask_xy
        self.de_d = n_de_d
        self.marq_pos = n_pos & self.mask_pos
        self.frame_div = n_div & self.mask_div


class _BrokenCellSlice(Marquee):
    """cell_idx = u[9:4] instead of u[9:5]: one slice bit short, so every
    cell reports the glyph column of its neighbour as well."""

    def __init__(self, *a, **kw):
        Marquee.__init__(self, *a, **kw)
        hi, lo = self.cell_slice
        self.cell_slice = (hi, lo - 1)


class _BrokenRowGate(Marquee):
    """text_on without the full-width in_text_rows compare.

    Pass C shows this changes nothing on today's geometry -- the gate is
    defence in depth, not load-bearing -- so it is a probe there, not a control.
    """

    def __init__(self, *a, **kw):
        Marquee.__init__(self, *a, **kw)
        self.gate_rows = False


class _BrokenRowOffset(Marquee):
    """row = y_pos[4:0] with the ROW_LSB subtraction dropped: the glyph slides
    down four rows and its bottom four rows go blank."""

    def __init__(self, *a, **kw):
        Marquee.__init__(self, *a, **kw)
        self.row_sub = 0


class _BrokenScrollEveryFrame(Marquee):
    """marq_pos advances every frame instead of every SCROLL_FRAME_DIV frames."""

    def __init__(self, *a, **kw):
        Marquee.__init__(self, *a, **kw)
        self.frame_div_last = 0


class _BrokenIgnoreEn(Marquee):
    """I_en dropped from in_band: SW4 can no longer hide the banner."""

    def __init__(self, *a, **kw):
        Marquee.__init__(self, *a, **kw)
        self.en_gates_band = False


class _BrokenBitOrder(Marquee):
    """glyph_bits[gcol] instead of glyph_bits[CELL-1-gcol]: mirrored text."""

    def __init__(self, *a, **kw):
        Marquee.__init__(self, *a, **kw)
        self.bit_order_reversed = True


# ---------------------------------------------------------------------------
# Rasters
# ---------------------------------------------------------------------------
def raster_real(frames, cfg):
    """The real color_bar waveform: 640 active + 160 blank, 480 lines + 45."""
    h_active = cfg["H_ACTIVE"]
    h_blank = H_TOTAL - h_active
    v_blank_lines = V_TOTAL - cfg["V_ACTIVE"]
    for _ in range(frames):
        for _ in range(cfg["V_ACTIVE"]):
            for _ in range(h_active):
                yield 1
            for _ in range(h_blank):
                yield 0
        for _ in range(v_blank_lines * H_TOTAL):
            yield 0


def raster_line(frames, cfg, active=None, blank=1):
    """Same line/line structure, shortened blanking. Pass A proves equivalence."""
    h_active = cfg["H_ACTIVE"] if active is None else active
    for _ in range(frames):
        for _ in range(cfg["V_ACTIVE"]):
            for _ in range(h_active):
                yield 1
            for _ in range(blank):
                yield 0
        yield 0


def run_raster(m, gen_de, bg=0x000000, en=1, probe=None):
    """Drive the model; probe(de, out, diag) is called before every step()."""
    for de in gen_de:
        out, diag = m.comb(de, bg, en)
        if probe is not None:
            probe(de, out, diag)
        m.step(de)
    return m


def wrap_ticks(m, cfg, count):
    """Apply `count` frame_wrap events directly.

    Pass A proves each real frame produces exactly one frame_wrap, on the first
    blanking cycle after the last line, with y_pos == V_ACTIVE-1. So driving
    that single state is equivalent to a whole frame as far as the scroll state
    machine is concerned -- and it lets the full TRAVEL walk run in seconds.
    """
    seq = []
    for _ in range(count):
        m.preset(x_pos=0, y_pos=cfg["V_ACTIVE_W"] - 1, de_d=1,
                 marq_pos=m.marq_pos, frame_div=m.frame_div)
        m.step(0)
        seq.append((m.marq_pos, m.frame_div))
    return seq


# ---------------------------------------------------------------------------
# Pass A -- raster tracker
# ---------------------------------------------------------------------------
def test_pass_a(m, cfg):
    print("\n[A] raster tracker vs. the real 640x480 raster")

    missing = check_always_shape(strip_comments(read(RTL)))
    check(not missing, "the always block still matches the model line for line",
          "missing fragments: %s" % missing)
    if missing:
        return

    def scan(gen_de, tag, frames):
        bad_x = bad_y = bad_de_d = bad_blank = 0
        first_bad = None
        wraps = 0
        line_starts = []
        pixel = 0
        line = 0
        seen_lines = 0

        def probe(de, prev_de, out, diag):
            nonlocal bad_x, bad_y, bad_de_d, bad_blank, first_bad, wraps
            nonlocal pixel, line, seen_lines
            if diag["de_d_expect"] != m.de_d:
                bad_de_d += 1
                first_bad = first_bad or ("de_d", line, pixel)
            if de:
                if prev_de == 0:
                    # first active cycle of a line: state must be pristine
                    line_starts.append((m.x_pos, m.y_pos, m.de_d, line))
                    pixel = 0
                if m.x_pos != pixel:
                    bad_x += 1
                    first_bad = first_bad or ("x_pos", line, pixel, m.x_pos)
                if m.y_pos != line:
                    bad_y += 1
                    first_bad = first_bad or ("y_pos", line, pixel, m.y_pos)
                pixel += 1
            else:
                if m.x_pos != 0:
                    bad_blank += 1
                    first_bad = first_bad or ("blanking x_pos", line, pixel)
                if prev_de == 1:
                    seen_lines += 1
                    line += 1
                    if line == cfg["V_ACTIVE"]:
                        line = 0
            if diag["frame_wrap"]:
                wraps += 1

        # de_d must be I_de delayed by exactly one clock
        holder = {"prev": 0}

        def wrapped_probe(de, out, diag):
            prev = holder["prev"]
            holder["prev"] = de
            diag["de_d_expect"] = prev
            probe(de, prev, out, diag)

        run_raster(m, gen_de, probe=wrapped_probe)
        return {"tag": tag, "bad_x": bad_x, "bad_y": bad_y,
                "bad_de_d": bad_de_d, "bad_blank": bad_blank,
                "first_bad": first_bad, "wraps": wraps,
                "lines": seen_lines, "line_starts": line_starts,
                "frames": frames}

    m.reset()
    real = scan(raster_real(1, cfg), "real 800x525 timing", 1)
    m.reset()
    short = scan(raster_line(2, cfg), "1-cycle blanking", 2)

    for r in (real, short):
        tag = r["tag"]
        check(r["bad_x"] == 0, "%s: x_pos == active pixel index 0..%d every cycle"
              % (tag, cfg["H_ACTIVE"] - 1), "first mismatch %s" % (r["first_bad"],))
        check(r["bad_y"] == 0, "%s: y_pos == line index 0..%d every cycle"
              % (tag, cfg["V_ACTIVE"] - 1), "first mismatch %s" % (r["first_bad"],))
        check(r["bad_de_d"] == 0, "%s: de_d is I_de delayed one clock" % tag,
              "first mismatch %s" % (r["first_bad"],))
        check(r["bad_blank"] == 0, "%s: x_pos is 0 throughout blanking" % tag,
              "first mismatch %s" % (r["first_bad"],))
        check(r["lines"] == r["frames"] * cfg["V_ACTIVE"],
              "%s: saw %d falling de edges = %d lines"
              % (tag, r["lines"], r["frames"] * cfg["V_ACTIVE"]))
        check(r["wraps"] == r["frames"],
              "%s: exactly one frame_wrap per frame (%d)" % (tag, r["wraps"]),
              "got %d" % r["wraps"])

    bad_starts = [(x, y, d, l) for (x, y, d, l) in real["line_starts"]
                  if (x, y, d) != (0, l, 0)]
    check(not bad_starts,
          "at the first active cycle of a line the state is exactly "
          "(x_pos=0, y_pos=line, de_d=0) -- this is what Pass D presets",
          "%d lines disagree, e.g. %s" % (len(bad_starts), bad_starts[:3]))

    # Scroll cadence over real frames: pos = frames // SCROLL_FRAME_DIV.
    div = cfg["SCROLL_FRAME_DIV"]
    frames = 2 * div + 2
    want_pos = [w // div for w in range(1, frames + 1)]

    m.reset()
    got = []
    for de in raster_line(frames, cfg):
        out, diag = m.comb(de, 0, 1)
        m.step(de)
        if diag["frame_wrap"]:
            got.append((m.marq_pos, m.frame_div))
    check(len(got) == frames,
          "%d frames produced %d frame_wrap events" % (frames, len(got)),
          "got %d" % len(got))
    check([p for p, _ in got] == want_pos,
          "over %d real frames marq_pos walks %s (1 px every %d frames)"
          % (frames, want_pos, div),
          "got %s" % [p for p, _ in got])
    check([d for _, d in got] == [w % div for w in range(1, frames + 1)],
          "frame_div is the scroll divider counting 0..%d" % (div - 1),
          "got %s" % [d for _, d in got])


# ---------------------------------------------------------------------------
# Pass B -- the borrow-wrap addressing trick, exhaustively
# ---------------------------------------------------------------------------
def test_pass_b(m, cfg):
    print("\n[B] u = x_pos + marq_pos - H_ACTIVE on an 11 bit wire, all %d pairs"
          % (cfg["H_ACTIVE"] * cfg["TRAVEL"]))
    h = cfg["H_ACTIVE"]
    text_w = cfg["TEXT_W"]
    pitch = cfg["PITCH"]
    n_cells = cfg["N_CELLS"]
    gutter, gutter_last = cfg["GUTTER_W"], cfg["GUTTER_LAST_W"]

    bad_region = bad_u = bad_cell = bad_col = bad_gutter = 0
    max_cell = -1
    first = None
    for pos in range(cfg["TRAVEL"]):
        for x in range(h):
            s, u, in_region, cell, col, col_in_glyph, gcol = m.addressing(x, pos)
            total = x + pos                      # unbounded reference
            want_region = h <= total < h + text_w
            if in_region != want_region:
                bad_region += 1
                first = first or ("in_region", x, pos, u)
            if u != (total - h) & m.mask_pos:
                bad_u += 1
                first = first or ("u", x, pos, u)
            if want_region:
                rel = total - h
                if cell != rel // pitch:
                    bad_cell += 1
                    first = first or ("cell", x, pos, cell)
                if col != rel % pitch:
                    bad_col += 1
                    first = first or ("col", x, pos, col)
                if col_in_glyph != (gutter <= col <= gutter_last):
                    bad_gutter += 1
                    first = first or ("col_in_glyph", x, pos, col)
                if cell > max_cell:
                    max_cell = cell

    check(bad_region == 0,
          "in_region == (640 <= x+pos < 1120) for every pair: the single "
          "unsigned compare covers both ends of the travel",
          "%d disagreements, first %s" % (bad_region, first))
    check(bad_u == 0,
          "u equals (x+pos-640) wrapped onto the %d bit wire" % m.widths["u"],
          "%d disagreements, first %s" % (bad_u, first))
    check(bad_cell == 0, "cell_idx = u[%d:%d] is the exact pitch quotient"
          % m.cell_slice, "%d disagreements, first %s" % (bad_cell, first))
    check(bad_col == 0, "col = u[%d:%d] is the exact pitch remainder"
          % m.col_slice, "%d disagreements, first %s" % (bad_col, first))
    check(bad_gutter == 0,
          "col_in_glyph keeps the %d px gutter on both sides of each cell"
          % gutter, "%d disagreements, first %s" % (bad_gutter, first))
    cell_arg_msb = m.ops["cell_arg_mask"].bit_length() - 1
    check(max_cell == n_cells - 1,
          "cell_idx peaks at %d, so marquee_glyph's cell_idx[%d:0] slice "
          "cannot alias" % (n_cells - 1, cell_arg_msb),
          "max cell seen was %d" % max_cell)
    check(m.ops["cell_arg_mask"] >= n_cells - 1,
          "cell_idx[%d:0] can address all %d cells" % (cell_arg_msb, n_cells))


# ---------------------------------------------------------------------------
# Pass C -- geometry
# ---------------------------------------------------------------------------
def test_pass_c(m, cfg, ops):
    print("\n[C] band geometry and its neighbours")

    check(cfg["BAND_Y"] == (cfg["V_ACTIVE"] - cfg["BAND_H"]) // 2,
          "the band is centred: BAND_Y=%d, centre row y=%d"
          % (cfg["BAND_Y"], cfg["BAND_Y"] + cfg["BAND_H"] // 2),
          "centre is %d, not %d" % (cfg["BAND_Y"] + cfg["BAND_H"] // 2,
                                    cfg["V_ACTIVE"] // 2))
    check(cfg["BAND_Y"] + cfg["BAND_H"] // 2 == cfg["V_ACTIVE"] // 2,
          "band centre row == V_ACTIVE/2 == %d" % (cfg["V_ACTIVE"] // 2))
    check(cfg["BAND_H"] == 1 + 3 + cfg["CELL"] + 3 + 1,
          "BAND_H=%d is 1 px edge + 3 px padding + %d px glyph + 3 px padding "
          "+ 1 px edge" % (cfg["BAND_H"], cfg["CELL"]))
    check(cfg["BAND_TEXT_Y"] == cfg["BAND_Y"] + 4,
          "glyph rows start 4 rows below the band top (BAND_TEXT_Y=%d)"
          % cfg["BAND_TEXT_Y"])
    check(cfg["BAND_TEXT_Y_LAST"] == cfg["BAND_TEXT_Y"] + cfg["CELL"] - 1,
          "glyph rows end at %d" % cfg["BAND_TEXT_Y_LAST"])
    check(cfg["ROW_LSB"] == cfg["BAND_TEXT_Y"] % cfg["PITCH"],
          "ROW_LSB=%d == BAND_TEXT_Y %% PITCH, so y_pos[4:0]-ROW_LSB is the "
          "glyph row" % cfg["ROW_LSB"])

    osd_lo, osd_hi = parse_panel(OSD_RTL, "OSD")
    vis_lo, vis_hi = parse_panel(VIS_RTL, "spectrum")
    check(cfg["BAND_Y"] > osd_hi,
          "no overlap with the OSD panel (y %d..%d vs band y %d..%d)"
          % (osd_lo, osd_hi, cfg["BAND_Y"], cfg["BAND_Y_LAST"]),
          "band starts at %d, OSD ends at %d" % (cfg["BAND_Y"], osd_hi))
    check(cfg["BAND_Y_LAST"] < vis_lo,
          "no overlap with the spectrum panel (y %d..%d)" % (vis_lo, vis_hi),
          "band ends at %d, spectrum starts at %d" % (cfg["BAND_Y_LAST"], vis_lo))
    check(cfg["BAND_Y"] > 0 and cfg["BAND_Y_LAST"] < cfg["V_ACTIVE"],
          "the band fits inside the active picture")

    check(tuple(ops["edge_rows"]) == (cfg["BAND_Y"], cfg["BAND_Y_LAST"]),
          "the 1 px cyan edges land on the first and last band row (%s)"
          % (ops["edge_rows"],))
    check(ops["band_lo"] == cfg["BAND_Y"] and ops["band_hi"] == cfg["BAND_Y_LAST"],
          "in_band uses the full-width y compare over %d..%d"
          % (ops["band_lo"], ops["band_hi"]))
    check(ops["text_rows"] == (cfg["BAND_TEXT_Y"], cfg["BAND_TEXT_Y_LAST"]),
          "in_text_rows uses the full-width y compare over %d..%d"
          % ops["text_rows"])

    # The 5 bit row slice repeats every 32 rows; only the full-width compare
    # stops those repeats from lighting up. Enumerate every row of the picture.
    alias_in = []
    row_bad = []
    for y in range(cfg["V_ACTIVE"]):
        r = m._slice(y, *ops["row_slice"])
        r = (r - ops["row_sub"]) & m.mask5
        inside = ops["text_rows"][0] <= y <= ops["text_rows"][1]
        if inside and r != y - cfg["BAND_TEXT_Y"]:
            row_bad.append((y, r))
        if r < cfg["CELL"] and not inside:
            alias_in.append(y)
    check(not row_bad,
          "inside the text rows the 5 bit slice yields exactly y-BAND_TEXT_Y "
          "(0..%d)" % (cfg["CELL"] - 1),
          "%d rows disagree, e.g. %s" % (len(row_bad), row_bad[:5]))
    check(alias_in,
          "%d rows elsewhere in the picture alias to a valid glyph row; in_band "
          "is what keeps them dark" % len(alias_in))

    # Inside the band the padding rows alias to glyph rows ABOVE the box, which
    # the .vh always leaves zero (its case only covers rows 0..CELL-1). That is
    # what makes in_text_rows defence in depth rather than load-bearing -- and
    # this check is the one that fails if CELL ever grows into the padding.
    padding = []
    for y in range(cfg["BAND_Y"], cfg["BAND_Y_LAST"] + 1):
        if y in ops["edge_rows"]:
            continue
        if ops["text_rows"][0] <= y <= ops["text_rows"][1]:
            continue
        padding.append((y, (m._slice(y, *ops["row_slice"]) - ops["row_sub"])
                        & m.mask5))
    over = [(y, r) for y, r in padding if r < cfg["CELL"]]
    check(not over,
          "the %d padding rows alias to glyph rows %s, all >= CELL=%d and "
          "therefore always zero"
          % (len(padding), sorted({r for _y, r in padding}), cfg["CELL"]),
          "%s would address live glyph rows -- in_text_rows is now load-bearing"
          % over)

    def band_of(model):
        return capture_band(model, cfg, 700, lambda x, y: (0xFF, 0xFF, 0xFF))

    ungated = band_of(_BrokenRowGate(cfg, m.widths, ops, m.font))
    gated = band_of(m)
    same = all(a[0] == b[0] for y in gated for a, b in zip(gated[y], ungated[y]))
    check(same,
          "dropping in_text_rows changes no pixel today, so the gate is free "
          "insurance rather than the only thing keeping the padding dark")

    check(cfg["PITCH"] & (cfg["PITCH"] - 1) == 0,
          "PITCH=%d is a power of two, so cell/col are bit slices, not a "
          "division" % cfg["PITCH"])
    check(cfg["H_ACTIVE"] % cfg["PITCH"] == 0,
          "H_ACTIVE=%d is a multiple of PITCH, so u = s - H_ACTIVE stays "
          "cell-aligned" % cfg["H_ACTIVE"])
    check(cfg["TRAVEL"] == cfg["H_ACTIVE"] + cfg["TEXT_W"],
          "TRAVEL=%d takes the text from fully off-right to fully off-left"
          % cfg["TRAVEL"])
    check(cfg["TRAVEL"] - 1 <= m.mask_pos,
          "TRAVEL-1=%d fits in the %d bit marq_pos" % (cfg["TRAVEL"] - 1,
                                                       m.widths["marq_pos"]))
    check(cfg["H_ACTIVE"] - 1 + cfg["TRAVEL"] - 1 <= m.mask_pos,
          "the widest s = x_pos + marq_pos = %d fits in %d bits"
          % (cfg["H_ACTIVE"] - 1 + cfg["TRAVEL"] - 1, m.widths["s"]))

    # The parsed RTL must agree with the generator, or Pass D's golden
    # reference (gen.band_pixel) would be comparing against the wrong geometry.
    mismatches = []
    for name in ("CELL", "PITCH", "GUTTER", "N_CELLS", "H_ACTIVE", "V_ACTIVE",
                 "TEXT_W", "TRAVEL", "BAND_H", "BAND_Y", "BAND_TEXT_Y",
                 "DIM_SHIFT"):
        rtl_val = cfg[name]
        gen_val = getattr(gen, name)
        if rtl_val != gen_val:
            mismatches.append("%s: RTL %s vs generator %s" % (name, rtl_val, gen_val))
    check(not mismatches,
          "marquee_overlay.v's localparams agree with gen_marquee_font.py",
          "; ".join(mismatches))
    check(ops["text_rgb"] == (gen.TEXT_RGB[0] << 16 | gen.TEXT_RGB[1] << 8
                              | gen.TEXT_RGB[2]),
          "text colour 24'h%06X matches the generator" % ops["text_rgb"])
    check(ops["edge_rgb"] == (gen.EDGE_RGB[0] << 16 | gen.EDGE_RGB[1] << 8
                              | gen.EDGE_RGB[2]),
          "edge colour 24'h%06X matches the generator" % ops["edge_rgb"])


# ---------------------------------------------------------------------------
# Pass D -- whole-band render against the golden pixel model
# ---------------------------------------------------------------------------
def rgb_int(rgb):
    """(r, g, b) -> the packed 24-bit value the RTL sees on I_rgb."""
    return (rgb[0] << 16) | (rgb[1] << 8) | rgb[2]


def capture_band(m, cfg, pos, bg_fn, en=1):
    """Run BAND_H real lines from the top of the band, capturing every pixel."""
    m.preset(x_pos=0, y_pos=cfg["BAND_Y_W"], de_d=0, marq_pos=pos)
    rows = {}
    for _ in range(cfg["BAND_H"]):
        y = m.y_pos
        row = []
        for x in range(cfg["H_ACTIVE"]):
            out, diag = m.comb(1, rgb_int(bg_fn(x, y)), en)
            row.append((out, diag))
            m.step(1)
        m.step(0)                      # falling edge: y_pos advances
        rows[y] = row
    return rows


def test_pass_d(m, cfg, font):
    print("\n[D] whole-band render vs. gen_marquee_font.band_pixel")

    white = (0xFF, 0xFF, 0xFF)
    black = (0, 0, 0)
    positions = [0, 1, 5, 32, 240, 639, 640, 700, 820, 940, cfg["TRAVEL"] - 1]
    backgrounds = (("white", lambda x, y: white),
                   ("black", lambda x, y: black),
                   ("gradient", lambda x, y: (x * 255 // 639, y * 255 // 479,
                                              (x + y) % 256)))

    total_pixels = 0
    for pos in positions:
        for bg_name, bg_fn in backgrounds:
            rows = capture_band(m, cfg, pos, bg_fn)
            bad = []
            for y in sorted(rows):
                for x, (out, _diag) in enumerate(rows[y]):
                    want = gen.band_pixel(x, y, pos, bg_fn(x, y), font)
                    want_int = rgb_int(want)
                    total_pixels += 1
                    if out != want_int:
                        if len(bad) < 5:
                            bad.append("(x=%d,y=%d) got 24'h%06X want 24'h%06X"
                                       % (x, y, out, want_int))
            check(not bad,
                  "marq_pos=%4d on %-8s: all %d band pixels match the golden "
                  "model" % (pos, bg_name, cfg["BAND_H"] * cfg["H_ACTIVE"]),
                  "%s ..." % ", ".join(bad))

    print("    (%d pixel comparisons)" % total_pixels)

    # Every column of the band is dimmed except text and the two edge rows.
    rows = capture_band(m, cfg, 700, lambda x, y: white)
    dim_want = 0xFF >> cfg["DIM_SHIFT"]
    dim_rgb = (dim_want << 16) | (dim_want << 8) | dim_want
    interior = [out for y in sorted(rows)
                if y not in (cfg["BAND_Y"], cfg["BAND_Y_LAST"])
                for out, _ in rows[y]]
    offenders = sorted({out for out in interior
                        if out not in (m.ops["text_rgb"], dim_rgb)})
    check(not offenders,
          "on a white picture every non-text interior pixel is the picture "
          ">> %d (24'h%06X) or the text colour" % (cfg["DIM_SHIFT"], dim_rgb),
          "also found %s" % ["24'h%06X" % o for o in offenders[:5]])
    edge_rows = [out for y in (cfg["BAND_Y"], cfg["BAND_Y_LAST"])
                 for out, _ in rows[y]]
    check(all(out == m.ops["edge_rgb"] for out in edge_rows),
          "both 1 px edge rows are solid 24'h%06X across all %d columns"
          % (m.ops["edge_rgb"], cfg["H_ACTIVE"]))

    # Outside the band the module must be invisible.
    untouched = 0
    for y in (cfg["BAND_Y_W"] - 1, cfg["BAND_Y_LAST_W"] + 1):
        m.preset(x_pos=0, y_pos=y, de_d=1, marq_pos=700)
        for x in range(cfg["H_ACTIVE"]):
            out, diag = m.comb(1, 0xABCDEF, 1)
            if out == 0xABCDEF and not diag["in_band"]:
                untouched += 1
            m.step(1)
    check(untouched == 2 * cfg["H_ACTIVE"],
          "the rows just above and below the band pass I_rgb through untouched "
          "(%d pixels)" % untouched, "got %d" % untouched)

    # SW4 mask: with I_en low the module must be a wire.
    rows = capture_band(m, cfg, 700, lambda x, y: (0x11, 0x22, 0x33), en=0)
    leaks = [(x, y, out) for y in sorted(rows) for x, (out, _d) in enumerate(rows[y])
             if out != 0x112233]
    check(not leaks, "with I_en=0 (SW4 ON) the band is bypassed pixel for pixel",
          "%d pixels changed, e.g. %s" % (len(leaks), leaks[:3]))

    # Scroll walk across the whole travel.
    m.reset()
    div = cfg["SCROLL_FRAME_DIV"]
    seq = wrap_ticks(m, cfg, cfg["TRAVEL"] * div)
    got_pos = [p for p, _ in seq]
    want_pos = [w // div for w in range(1, cfg["TRAVEL"] * div + 1)]
    # the last step of the walk wraps back to 0
    want_pos[-1] = 0
    check(got_pos == want_pos,
          "%d frame_wraps walk marq_pos 0..%d and back to 0"
          % (cfg["TRAVEL"] * div, cfg["TRAVEL"] - 1),
          "first divergence at wrap %d: got %d want %d"
          % next(((i + 1, g, w) for i, (g, w) in enumerate(zip(got_pos, want_pos))
                  if g != w), (0, 0, 0)))
    check(max(p for p, _ in seq) == cfg["TRAVEL"] - 1,
          "marq_pos never exceeds TRAVEL-1=%d" % (cfg["TRAVEL"] - 1),
          "peaked at %d" % max(p for p, _ in seq))
    check(seq[-1] == (0, 0),
          "after a whole loop the scroll state is back at (pos=0, div=0), so "
          "the pattern repeats exactly")

    # The wrap primitive and the real raster must agree frame for frame.
    m.reset()
    real_seq = wrap_ticks(m, cfg, 8)
    m.reset()
    raster_seq = []
    for de in raster_line(8, cfg):
        _out, diag = m.comb(de, 0, 1)
        m.step(de)
        if diag["frame_wrap"]:
            raster_seq.append((m.marq_pos, m.frame_div))
    check(real_seq == raster_seq,
          "the frame_wrap primitive Pass D uses for the long walk produces the "
          "same (marq_pos, frame_div) sequence as 8 real frames",
          "%s vs %s" % (real_seq, raster_seq))

    fps = 25e6 / (H_TOTAL * V_TOTAL)
    print("    scroll speed: %.2f px/s at %.2f Hz, one loop every %.1f s"
          % (fps / cfg["SCROLL_FRAME_DIV"], fps,
             cfg["TRAVEL"] * cfg["SCROLL_FRAME_DIV"] / fps))


# ---------------------------------------------------------------------------
# Pass E -- negative controls
# ---------------------------------------------------------------------------
def test_pass_e(cfg, widths, ops, font):
    print("\n[E] negative controls")

    def band_matches(cls, pos=700):
        m = cls(cfg, widths, ops, font)
        rows = capture_band(m, cfg, pos, lambda x, y: (0xFF, 0xFF, 0xFF))
        bad = 0
        for y in sorted(rows):
            for x, (out, _diag) in enumerate(rows[y]):
                want = gen.band_pixel(x, y, pos, (0xFF, 0xFF, 0xFF), font)
                if out != rgb_int(want):
                    bad += 1
        return bad

    good = band_matches(Marquee)
    check(good == 0, "the reference model has zero mismatches to beat",
          "%d mismatches" % good)

    expect_fail(band_matches(_BrokenCellSlice) == 0,
                "E1 cell_idx = u[9:4] (one slice bit short) changes the "
                "rendered band")
    offset_bad = band_matches(_BrokenRowOffset)
    expect_fail(offset_bad == 0,
                "E2 dropping the ROW_LSB subtraction slides the glyph down four "
                "rows (%d band pixels differ)" % offset_bad)
    expect_fail(band_matches(_BrokenBitOrder) == 0,
                "E3 glyph_bits[gcol] instead of [23-gcol] mirrors the text")

    div = cfg["SCROLL_FRAME_DIV"]
    good_m = Marquee(cfg, widths, ops, font)
    bad_m = _BrokenScrollEveryFrame(cfg, widths, ops, font)
    good_seq = [p for p, _ in wrap_ticks(good_m, cfg, 4 * div)]
    bad_seq = [p for p, _ in wrap_ticks(bad_m, cfg, 4 * div)]
    expect_fail(good_seq == bad_seq,
                "E4 advancing marq_pos every frame doubles the scroll speed "
                "(%s vs %s over %d frames)" % (good_seq, bad_seq, 4 * div))

    en_off = _BrokenIgnoreEn(cfg, widths, ops, font)
    rows = capture_band(en_off, cfg, 700, lambda x, y: (0x11, 0x22, 0x33), en=0)
    changed = sum(1 for y in sorted(rows) for out, _ in rows[y] if out != 0x112233)
    expect_fail(changed == 0,
                "E5 ignoring I_en leaves the banner on screen when SW4 masks it "
                "(%d pixels still drawn)" % changed)


# ---------------------------------------------------------------------------
def main():
    print("=" * 72)
    print("marquee_overlay.v cycle-accurate model")
    print("=" * 72)
    cfg, widths, ops, raw = parse_rtl()
    font = load_font()
    print("parsed from RTL: %d cells of %d px on a %d px pitch, band y %d..%d, "
          "glyph rows y %d..%d, TRAVEL %d, 1 px per %d frames"
          % (cfg["N_CELLS"], cfg["CELL"], cfg["PITCH"], cfg["BAND_Y"],
             cfg["BAND_Y_LAST"], cfg["BAND_TEXT_Y"], cfg["BAND_TEXT_Y_LAST"],
             cfg["TRAVEL"], cfg["SCROLL_FRAME_DIV"]))
    print("font table     : %s (%d cells, %d non-zero rows)"
          % (os.path.basename(VH), len(font),
             sum(1 for c in font.values() for b in c.values() if b)))

    m = Marquee(cfg, widths, ops, font)

    test_pass_a(m, cfg)
    test_pass_b(m, cfg)
    test_pass_c(m, cfg, ops)
    test_pass_d(m, cfg, font)
    test_pass_e(cfg, widths, ops, font)

    print("\n" + "=" * 72)
    if FAILURES:
        print("FAILED %d of %d checks:" % (len(FAILURES), CHECKS[0]))
        for f in FAILURES:
            print("  - %s" % f)
        return 1
    print("ALL %d CHECKS PASSED" % CHECKS[0])
    return 0


if __name__ == "__main__":
    sys.exit(main())
