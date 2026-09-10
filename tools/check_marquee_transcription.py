#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Cross-check the marquee RTL, its generated font and the top-level wiring.

sim_marquee.py proves marquee_overlay.v *behaves*. This file proves it is still
the design that was intended, and that nothing around it drifted:

  1. marquee_font.vh on disk is byte-identical to what gen_marquee_font.py
     emits for a fresh render of the slogan, and that fresh render still passes
     the generator's own quality gates (ink, centring, clipping, duplicates).
  2. Every localparam, bit slice, register width and colour in
     marquee_overlay.v is re-derived from the generator's constants -- not
     compared against numbers typed a second time in this file.
  3. The instance in top_tf_hdmi_audio.v connects every port exactly once and
     sits at the very end of the video chain:
     osd_overlay.O_rgb -> vout_data_osd -> marquee_overlay -> vout_data ->
     video_rgb_to_axis_640x480.I_rgb.
  4. SW4 polarity: marquee_en = sw4_v1 (OFF = banner shown), sw[3] on its own
     2-FF chain, the SW1-3 path (sw_v0 <= sw[2:0], trans_mode = ~sw_v1)
     untouched, and sw[3] still PULLUP in pin.adc so OFF really reads 1.
  5. Build files: marquee_overlay.v registered in the .al; marquee_font.vh
     deliberately NOT registered and reached by a `include inside the module;
     and no declared identifier collides with a Verilog-2001 reserved word.
  6. Nine in-memory mutations, each of which the checks above must catch.

The RTL parsing is imported from sim_marquee so the two tools cannot disagree
about what the Verilog says.

Run from anywhere:  python tools/check_marquee_transcription.py
"""
import contextlib
import io
import math
import os
import re
import sys

TOOLS = os.path.dirname(os.path.abspath(__file__))
if TOOLS not in sys.path:
    sys.path.insert(0, TOOLS)

import gen_marquee_font as gen
import sim_marquee as sim

REPO = os.path.dirname(TOOLS)
HDL = os.path.join(REPO, "src", "user_source", "hdl_source")
RTL = os.path.join(HDL, "marquee_overlay.v")
VH = gen.VH_PATH
TOP = os.path.join(HDL, "top_tf_hdmi_audio.v")
AL = os.path.join(REPO, "src", "td_project", "HDMI1.4b_Transmitter_v1.0.al")
ADC = os.path.join(REPO, "src", "user_source", "constraints_source", "pin.adc")

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


class Sources(object):
    """The five files this checker reads, so controls can mutate one in memory."""

    def __init__(self, rtl=None, vh=None, top=None, al=None, adc=None):
        self.rtl = read(RTL) if rtl is None else rtl
        self.vh = read(VH) if vh is None else vh
        self.top = read(TOP) if top is None else top
        self.al = read(AL) if al is None else al
        self.adc = read(ADC) if adc is None else adc

    def clone(self, **kw):
        return Sources(rtl=kw.get("rtl", self.rtl), vh=kw.get("vh", self.vh),
                       top=kw.get("top", self.top), al=kw.get("al", self.al),
                       adc=kw.get("adc", self.adc))


# ---------------------------------------------------------------------------
# Small structural helpers
# ---------------------------------------------------------------------------
def match_paren(text, open_idx):
    """Index of the ')' balancing the '(' at open_idx."""
    depth = 0
    for k in range(open_idx, len(text)):
        if text[k] == "(":
            depth += 1
        elif text[k] == ")":
            depth -= 1
            if depth == 0:
                return k
    raise ValueError("unbalanced parentheses at %d" % open_idx)


def find_instance(text, module):
    """(param_text or None, instance_name, port_text) for one instantiation.

    Paren-balanced rather than regex-greedy, because a parameter override like
    `#(.H_ACTIVE (640))` has nested parentheses.
    """
    m = re.search(r"\b%s\b\s*" % re.escape(module), text)
    if not m:
        return None, None, None
    i = m.end()
    params = None
    head = re.match(r"#\s*\(", text[i:])
    if head:
        open_idx = i + head.end() - 1
        close = match_paren(text, open_idx)
        params = text[open_idx + 1:close]
        i = close + 1
    tail = re.match(r"\s*(\w+)\s*\(", text[i:])
    if not tail:
        return params, None, None
    name = tail.group(1)
    open_idx = i + tail.end() - 1
    close = match_paren(text, open_idx)
    return params, name, text[open_idx + 1:close]


def connections(text):
    """`.port (signal)` pairs, in order, as a dict plus a duplicate list."""
    pairs = re.findall(r"\.\s*(\w+)\s*\(([^)]*)\)", text or "")
    seen, dupes = {}, []
    for port, sig in pairs:
        if port in seen:
            dupes.append(port)
        seen[port] = sig.strip()
    return seen, dupes


def module_ports(text, module):
    """{name: (direction, width)} from a module header."""
    m = re.search(r"module\s+%s\b.*?\)\s*\(" % re.escape(module), text, flags=re.S)
    if not m:
        return {}
    close = match_paren(text, m.end() - 1)
    body = text[m.end():close]
    ports = {}
    for direction, msb, lsb, name in re.findall(
            r"\b(input|output)\s+(?:wire|reg)?\s*"
            r"(?:\[\s*(\d+)\s*:\s*(\d+)\s*\]\s*)?(\w+)", body):
        width = 1 if msb is None or msb == "" else int(msb) - int(lsb) + 1
        ports[name] = (direction, width)
    return ports


def parse_rtl_text(text, tag="marquee_overlay.v"):
    code = sim.strip_comments(text)
    cfg = sim.parse_params(code, tag)
    widths = sim.parse_widths(code, tag)
    ops = sim.parse_ops(code, cfg)
    return cfg, widths, ops


def bits_to_log2(value):
    return int(math.ceil(math.log(value, 2))) if value > 1 else 0


# IEEE 1364-2001 reserved words. TD parses Verilog-2001, so a collision is a
# syntax error rather than a warning: `wire [4:0] cell;` alone cost a whole
# synthesis stage before this table existed.
RESERVED_WORDS = frozenset("""
    always and assign automatic begin buf bufif0 bufif1 case casex casez cell
    cmos config deassign default defparam design disable edge else end endcase
    endconfig endfunction endgenerate endmodule endprimitive endspecify
    endtable endtask event for force forever fork function generate genvar
    highz0 highz1 if ifnone incdir include initial inout input instance
    integer join large liblist library localparam macromodule medium module
    nand negedge nmos nor noshowcancelled not notif0 notif1 or output
    parameter pmos posedge primitive pull0 pull1 pulldown pullup
    pulsestyle_onevent pulsestyle_ondetect rcmos real realtime reg release
    repeat rnmos rpmos rtran rtranif0 rtranif1 scalared showcancelled signed
    small specify specparam strong0 strong1 supply0 supply1 table task time
    tran tranif0 tranif1 tri tri0 tri1 triand trior trireg unsigned use uwire
    var vectored wait wand weak0 weak1 while wire wor xnor xor
""".split())

_DECL_RE = re.compile(
    r"\b(?:wire|reg|input|output|inout|localparam|parameter|integer|genvar)\b"
    r"(?:\s*(?:wire|reg|signed|unsigned|integer|real|time))*"
    r"(?:\s*\[[^\]]*\])?"
    r"\s*(\w+)")
_SIBLING_RE = re.compile(r",\s*(?:\[[^\]]*\]\s*)?(\w+)")


def declared_names(code):
    """Every identifier a declaration introduces, plus module/function names."""
    names = set()
    matches = list(_DECL_RE.finditer(code))
    for i, m in enumerate(matches):
        names.add(m.group(1))
        # Bound the walk at the next declaration too: inside a port list there
        # is no semicolon for a long way and those commas separate declarations.
        stop = matches[i + 1].start() if i + 1 < len(matches) else len(code)
        tail = code[m.end():stop]
        semi = tail.find(";")
        for sib in _SIBLING_RE.finditer(tail if semi < 0 else tail[:semi]):
            names.add(sib.group(1))
    names.update(re.findall(r"\bfunction\b\s*(?:\[[^\]]*\]\s*)?(\w+)", code))
    names.update(re.findall(r"\bmodule\b\s+(\w+)", code))
    return names


# ---------------------------------------------------------------------------
# 1. The font
# ---------------------------------------------------------------------------
def check_font(src):
    print("\n[1] marquee_font.vh vs. a fresh render")

    rendered = gen.build_glyphs()
    cells = [rows for rows, _ in rendered]
    infos = [info for _, info in rendered]

    quality = gen.check_font(cells, infos)
    check(not quality,
          "a fresh render of %r still passes the generator's quality gates"
          % gen.SLOGAN, "; ".join(quality))

    expected = gen.emit_vh(cells)
    got = src.vh
    check(got.rstrip("\n") == expected.rstrip("\n"),
          "marquee_font.vh is byte-identical to gen_marquee_font.py's output "
          "(%d bytes, nobody edited it by hand)" % len(expected))

    diffs = gen.tables_equal(gen.parse_vh(got), gen.table_from_cells(cells))
    check(not diffs,
          "all %d case items (%d cells x %d rows) match bit for bit"
          % (gen.N_CELLS * gen.CELL, gen.N_CELLS, gen.CELL),
          "%d differing rows, first: %s" % (len(diffs), diffs[0] if diffs else ""))

    header = [l for l in got.splitlines()[:12] if l.startswith("// Slogan")]
    check(bool(header) and gen.SLOGAN in header[0],
          "the .vh header records the slogan it was rendered from",
          header[0] if header else "no slogan header line")
    check("%s @ %d px" % (gen.FONT_PATH, gen.FONT_SIZE) in got,
          "the .vh header records the font and size: %s @ %d px"
          % (gen.FONT_PATH, gen.FONT_SIZE))


# ---------------------------------------------------------------------------
# 2. Geometry transcription
# ---------------------------------------------------------------------------
def check_geometry(src):
    print("\n[2] marquee_overlay.v's constants re-derived from the generator")

    cfg, widths, ops = parse_rtl_text(src.rtl)

    derived = {
        "CELL": gen.CELL,
        "PITCH": gen.PITCH,
        "GUTTER": (gen.PITCH - gen.CELL) // 2,
        "N_CELLS": gen.N_CELLS,
        "H_ACTIVE": gen.H_ACTIVE,
        "V_ACTIVE": gen.V_ACTIVE,
        "TEXT_W": gen.N_CELLS * gen.PITCH,
        "TRAVEL": gen.H_ACTIVE + gen.TEXT_W,
        "BAND_H": 32,
        "BAND_Y": (gen.V_ACTIVE - 32) // 2,
        "BAND_TEXT_Y": (gen.V_ACTIVE - 32) // 2 + 4,
        "ROW_LSB": gen.BAND_TEXT_Y % gen.PITCH,
        "DIM_SHIFT": gen.DIM_SHIFT,
    }
    derived["BAND_Y_LAST"] = derived["BAND_Y"] + derived["BAND_H"] - 1
    derived["BAND_TEXT_Y_LAST"] = derived["BAND_TEXT_Y"] + derived["CELL"] - 1

    bad = []
    for name, want in sorted(derived.items()):
        if cfg.get(name) != want:
            bad.append("%s: RTL %s, derived %s" % (name, cfg.get(name), want))
    check(not bad, "the %d geometry localparams are all correct" % len(derived),
          "; ".join(bad))

    check(cfg["TEXT_W"] == gen.TEXT_W and cfg["TRAVEL"] == gen.TRAVEL,
          "TEXT_W=%d and TRAVEL=%d match the generator"
          % (gen.TEXT_W, gen.TRAVEL))

    # The width-typed copies must be the same numbers, otherwise a compare
    # silently truncates.
    copies = {"H_ACTIVE_W": "H_ACTIVE", "V_ACTIVE_W": "V_ACTIVE",
              "BAND_Y_W": "BAND_Y", "BAND_Y_LAST_W": "BAND_Y_LAST",
              "BAND_TEXT_Y_W": "BAND_TEXT_Y",
              "BAND_TEXT_Y_LAST_W": "BAND_TEXT_Y_LAST",
              "H_ACTIVE_P": "H_ACTIVE", "TEXT_W_P": "TEXT_W",
              "TRAVEL_LAST_P": None, "GUTTER_W": "GUTTER",
              "GUTTER_LAST_W": None, "ROW_LSB_W": "ROW_LSB",
              "FRAME_DIV_LAST": None}
    want_special = {"TRAVEL_LAST_P": cfg["TRAVEL"] - 1,
                    "GUTTER_LAST_W": cfg["GUTTER"] + cfg["CELL"] - 1,
                    "FRAME_DIV_LAST": cfg["SCROLL_FRAME_DIV"] - 1}
    bad = []
    for copy, base in copies.items():
        want = want_special[copy] if base is None else cfg[base]
        if cfg.get(copy) != want:
            bad.append("%s=%s should be %s" % (copy, cfg.get(copy), want))
    check(not bad, "the width-typed localparam copies carry the same values",
          "; ".join(bad))

    # Register widths: 11 bits must hold TRAVEL-1 and the widest s, 10 bits the
    # raster, and frame_div must hold FRAME_DIV_LAST.
    width_bad = []
    for name in ("s", "u", "marq_pos"):
        if widths[name] != gen.POS_BITS:
            width_bad.append("%s is %d bits, generator says %d"
                             % (name, widths[name], gen.POS_BITS))
    if 2 ** widths["marq_pos"] - 1 < cfg["TRAVEL"] - 1:
        width_bad.append("marq_pos cannot count to TRAVEL-1")
    if 2 ** widths["marq_pos"] - 1 < cfg["H_ACTIVE"] - 1 + cfg["TRAVEL"] - 1:
        width_bad.append("s overflows: x_pos + marq_pos needs more bits")
    for name in ("x_pos", "y_pos"):
        if 2 ** widths[name] - 1 < max(cfg["H_ACTIVE"], cfg["V_ACTIVE"]) - 1:
            width_bad.append("%s is too narrow for the raster" % name)
    if 2 ** widths["frame_div"] - 1 < cfg["FRAME_DIV_LAST"]:
        width_bad.append("frame_div cannot hold FRAME_DIV_LAST")
    check(not width_bad,
          "register widths are right (marq_pos/s/u %d bits, x_pos/y_pos %d bits, "
          "frame_div %d bits)" % (widths["marq_pos"], widths["x_pos"],
                                  widths["frame_div"]),
          "; ".join(width_bad))

    # Bit slices derived from PITCH, not restated.
    lo = bits_to_log2(gen.PITCH)
    need = bits_to_log2(gen.N_CELLS)
    chi, clo = ops["cell_slice"]
    slice_bad = []
    if clo != lo:
        slice_bad.append("cell_idx = u[%d:%d] must start at bit %d, the first bit "
                         "above the pitch field" % (chi, clo, lo))
    if chi < lo + need - 1:
        slice_bad.append("cell_idx = u[%d:%d] is too narrow to address %d cells"
                         % (chi, clo, gen.N_CELLS))
    if chi > gen.POS_BITS - 1:
        slice_bad.append("cell_idx = u[%d:%d] reaches past the %d bit wire"
                         % (chi, clo, gen.POS_BITS))
    if (gen.TEXT_W - 1) >> (chi + 1):
        slice_bad.append("cell_idx = u[%d:%d] keeps bits that can be set while "
                         "u < TEXT_W" % (chi, clo))
    if ops["col_slice"] != (lo - 1, 0):
        slice_bad.append("col = u[%d:%d], expected u[%d:0]"
                         % (ops["col_slice"][0], ops["col_slice"][1], lo - 1))
    if ops["row_slice"] != (lo - 1, 0):
        slice_bad.append("row uses y_pos[%d:%d], expected y_pos[%d:0]"
                         % (ops["row_slice"][0], ops["row_slice"][1], lo - 1))
    if ops["u_sub"] != gen.H_ACTIVE:
        slice_bad.append("u subtracts %d, expected H_ACTIVE=%d"
                         % (ops["u_sub"], gen.H_ACTIVE))
    if ops["region_lt"] != gen.TEXT_W:
        slice_bad.append("in_region compares against %d, expected TEXT_W=%d"
                         % (ops["region_lt"], gen.TEXT_W))
    if ops["col_lo"] != derived["GUTTER"] or ops["col_hi"] != derived["GUTTER"] + gen.CELL - 1:
        slice_bad.append("col_in_glyph window is %d..%d, expected %d..%d"
                         % (ops["col_lo"], ops["col_hi"], derived["GUTTER"],
                            derived["GUTTER"] + gen.CELL - 1))
    if ops["gcol_sub"] != derived["GUTTER"]:
        slice_bad.append("gcol subtracts %d, expected GUTTER=%d"
                         % (ops["gcol_sub"], derived["GUTTER"]))
    if ops["row_sub"] != derived["ROW_LSB"]:
        slice_bad.append("row subtracts %d, expected ROW_LSB=%d"
                         % (ops["row_sub"], derived["ROW_LSB"]))
    if ops["index_msb"] != gen.CELL - 1:
        slice_bad.append("the glyph bit index starts at %d, expected CELL-1=%d"
                         % (ops["index_msb"], gen.CELL - 1))
    if ops["index_width"] < bits_to_log2(gen.CELL):
        slice_bad.append("the glyph bit index is %d bits, needs %d"
                         % (ops["index_width"], bits_to_log2(gen.CELL)))
    if ops["cell_arg_mask"] < gen.N_CELLS - 1:
        slice_bad.append("marquee_glyph's cell_idx argument cannot address "
                         "%d cells" % gen.N_CELLS)
    check(not slice_bad, "every bit slice and operand is the one PITCH implies",
          "; ".join(slice_bad))

    window_bad = []
    if (ops["band_lo"], ops["band_hi"]) != (derived["BAND_Y"], derived["BAND_Y_LAST"]):
        window_bad.append("in_band covers %d..%d" % (ops["band_lo"], ops["band_hi"]))
    if tuple(ops["edge_rows"]) != (derived["BAND_Y"], derived["BAND_Y_LAST"]):
        window_bad.append("band_edge fires on %s" % (ops["edge_rows"],))
    if tuple(ops["text_rows"]) != (derived["BAND_TEXT_Y"], derived["BAND_TEXT_Y_LAST"]):
        window_bad.append("in_text_rows covers %d..%d" % ops["text_rows"])
    if ops["frame_wrap_y"] != gen.V_ACTIVE - 1:
        window_bad.append("frame_wrap fires at y=%d" % ops["frame_wrap_y"])
    if not ops["en_gates_band"]:
        window_bad.append("I_en does not gate in_band, so SW4 cannot hide the banner")
    check(not window_bad, "the row windows and I_en gating are as designed",
          "; ".join(window_bad))

    packed_text = (gen.TEXT_RGB[0] << 16) | (gen.TEXT_RGB[1] << 8) | gen.TEXT_RGB[2]
    packed_edge = (gen.EDGE_RGB[0] << 16) | (gen.EDGE_RGB[1] << 8) | gen.EDGE_RGB[2]
    check(ops["text_rgb"] == packed_text and ops["edge_rgb"] == packed_edge,
          "the output mux uses 24'h%06X for text and 24'h%06X for the edges"
          % (packed_text, packed_edge),
          "RTL has 24'h%06X / 24'h%06X" % (ops["text_rgb"], ops["edge_rgb"]))

    fps = 25e6 / (sim.H_TOTAL * sim.V_TOTAL)
    check(cfg["SCROLL_FRAME_DIV"] >= 1,
          "SCROLL_FRAME_DIV=%d -> %.2f px/s, one %.1f s loop at %.2f Hz"
          % (cfg["SCROLL_FRAME_DIV"], fps / cfg["SCROLL_FRAME_DIV"],
             cfg["TRAVEL"] * cfg["SCROLL_FRAME_DIV"] / fps, fps),
          "a divider below 1 would never advance")
    return cfg


# ---------------------------------------------------------------------------
# 3. Top-level wiring
# ---------------------------------------------------------------------------
CHAIN = [("audio_visualizer", "vout_data_audio"),
         ("osd_overlay", "vout_data_osd"),
         ("marquee_overlay", "vout_data")]


def check_wiring(src):
    print("\n[3] the instance in top_tf_hdmi_audio.v")

    top = sim.strip_comments(src.top)
    ports = module_ports(src.rtl, "marquee_overlay")
    check(sorted(ports) == ["I_clk", "I_de", "I_en", "I_rgb", "I_rst", "O_rgb"],
          "marquee_overlay declares exactly %s" % sorted(ports),
          "got %s" % sorted(ports))
    check(ports.get("I_rgb") == ("input", 24) and ports.get("O_rgb") == ("output", 24),
          "I_rgb and O_rgb are 24 bits wide")

    params, name, body = find_instance(top, "marquee_overlay")
    if not check(name is not None, "top instantiates marquee_overlay"):
        return
    check(name == "u_marquee_overlay", "the instance is named u_marquee_overlay",
          "got %s" % name)

    conn, dupes = connections(body)
    check(not dupes, "no port is connected twice", "duplicates: %s" % dupes)
    check(sorted(conn) == sorted(ports),
          "every port is connected exactly once (%d connections)" % len(conn),
          "missing %s, unexpected %s"
          % (sorted(set(ports) - set(conn)), sorted(set(conn) - set(ports))))

    wanted = {"I_clk": "video_clk", "I_rst": "rst_all", "I_de": "de",
              "I_en": "marquee_en", "I_rgb": "vout_data_osd", "O_rgb": "vout_data"}
    bad = ["%s is driven by %s, expected %s" % (p, conn.get(p), s)
           for p, s in sorted(wanted.items()) if conn.get(p) != s]
    check(not bad, "clock, reset, de, the SW4 mask and both pixel ports go where "
                   "the plan says", "; ".join(bad))

    pconn, _ = connections(params)
    pbad = ["%s=%s, expected %s" % (k, pconn.get(k), v)
            for k, v in (("H_ACTIVE", str(gen.H_ACTIVE)),
                         ("V_ACTIVE", str(gen.V_ACTIVE)))
            if pconn.get(k) != v]
    check(not pbad, "the parameter override matches the generator's resolution",
          "; ".join(pbad))

    # Chain order: each overlay's O_rgb must feed the next stage's I_rgb.
    for stage, (module, net) in enumerate(CHAIN):
        _p, inst, body = find_instance(top, module)
        if inst is None:
            check(False, "top instantiates %s" % module)
            continue
        conn, _ = connections(body)
        check(conn.get("O_rgb") == net,
              "chain %d/%d: %s drives %s" % (stage + 1, len(CHAIN), module, net),
              "%s.O_rgb is %s" % (module, conn.get("O_rgb")))
        if stage:
            prev_net = CHAIN[stage - 1][1]
            check(conn.get("I_rgb") == prev_net,
                  "chain %d/%d: %s reads %s" % (stage + 1, len(CHAIN), module, prev_net),
                  "%s.I_rgb is %s" % (module, conn.get("I_rgb")))

    _p, inst, body = find_instance(top, "video_rgb_to_axis_640x480")
    conn, _ = connections(body)
    check(inst is not None and conn.get("I_rgb") == "vout_data",
          "the marquee is the last overlay: vout_data feeds "
          "video_rgb_to_axis_640x480.I_rgb",
          "I_rgb is %s" % conn.get("I_rgb"))

    check(re.search(r"wire\s+\[\s*23\s*:\s*0\s*\]\s+vout_data_osd\s*;", top),
          "vout_data_osd is declared as a 24-bit wire")


# ---------------------------------------------------------------------------
# 4. SW4
# ---------------------------------------------------------------------------
SW4_FACTS = (
    (r"assign\s+marquee_en\s*=\s*sw4_v1\s*;",
     "marquee_en = sw4_v1, so SW4 OFF (pin high) shows the banner"),
    (r"assign\s+marquee_en\s*=\s*~\s*sw4_v1\s*;", None),   # must NOT exist
    (r"sw4_v0\s*<=\s*sw\[3\]\s*;", "sw4_v0 samples sw[3]"),
    (r"sw4_v1\s*<=\s*sw4_v0\s*;", "sw4_v1 completes the 2-FF synchroniser"),
    (r"sw4_v0\s*<=\s*1'b1\s*;", "sw4_v0 comes out of reset high (banner shown)"),
    (r"sw4_v1\s*<=\s*1'b1\s*;", "sw4_v1 comes out of reset high (banner shown)"),
    (r"reg\s+sw4_v0\s*;", "sw4_v0 is its own 1-bit register"),
    (r"reg\s+sw4_v1\s*;", "sw4_v1 is its own 1-bit register"),
    (r"wire\s+marquee_en\s*;", "marquee_en is declared"),
)

SW13_FACTS = (
    (r"sw_v0\s*<=\s*sw\[2:0\]\s*;", "SW1-3 still sample sw[2:0] only"),
    (r"sw_v1\s*<=\s*sw_v0\s*;", "the SW1-3 synchroniser is unchanged"),
    (r"assign\s+trans_mode\s*=\s*~\s*sw_v1\s*;", "trans_mode = ~sw_v1 is unchanged"),
    (r"sw_v0\s*<=\s*3'b111\s*;", "sw_v0 still resets to 3'b111"),
    (r"sw_v1\s*<=\s*3'b111\s*;", "sw_v1 still resets to 3'b111"),
    (r"reg\s+\[\s*2\s*:\s*0\s*\]\s*sw_v0\s*;", "sw_v0 is still 3 bits wide"),
    (r"reg\s+\[\s*2\s*:\s*0\s*\]\s*sw_v1\s*;", "sw_v1 is still 3 bits wide"),
    (r"input\s+\[\s*3\s*:\s*0\s*\]\s*sw\b", "the sw port is still 4 bits wide"),
)


def check_sw4(src):
    print("\n[4] SW4 polarity and the untouched SW1-3 path")

    top = sim.strip_comments(src.top)
    for pattern, label in SW4_FACTS:
        found = re.search(pattern, top) is not None
        if label is None:
            check(not found, "marquee_en is NOT inverted -- an inverted assign "
                             "would hide the banner at power-up")
        else:
            check(found, label, "pattern %s not found in the top" % pattern)

    for pattern, label in SW13_FACTS:
        check(re.search(pattern, top) is not None, label,
              "pattern %s not found in the top" % pattern)

    # The synchroniser must live in the same always block as the SW1-3 one, or
    # it would be sampled on a different edge.
    blocks = re.findall(r"always\s*@\s*\(posedge[^;]*?begin(.*?)end\s*$",
                        top, flags=re.S | re.M)
    same_block = any("sw4_v0 <= sw[3]" in re.sub(r"\s+", " ", b)
                     and "sw_v0 <= sw[2:0]" in re.sub(r"\s+", " ", b)
                     for b in blocks)
    check(same_block, "sw4_v0/sw4_v1 are clocked in the same always block as "
                      "sw_v0/sw_v1")

    m = re.search(r"set_pin_assignment\s*\{\s*sw\[3\]\s*\}\s*\{([^}]*)\}", src.adc)
    if check(m is not None, "pin.adc constrains sw[3]"):
        attrs = m.group(1)
        check("PULLTYPE = PULLUP" in attrs,
              "sw[3] is PULLUP, so SW4 OFF reads 1 and the banner shows at "
              "power-up", attrs.strip())


# ---------------------------------------------------------------------------
# 5. Build files
# ---------------------------------------------------------------------------
def check_build_files(src):
    print("\n[5] project registration, the `include, and identifier legality")

    m = re.search(r'<File Path="([^"]*marquee_overlay\.v)">(.*?)</File>',
                  src.al, flags=re.S)
    if check(m is not None, "marquee_overlay.v is registered in the .al"):
        body = m.group(2)
        check(m.group(1) == "../user_source/hdl_source/marquee_overlay.v",
              "the .al path is %s" % m.group(1))
        for attr, want in (("UsedInSyn", "true"), ("UsedInP&R", "true"),
                           ("BelongTo", "design_1")):
            check('Name="%s" Val="%s"' % (attr, want) in body,
                  '%s = %s for marquee_overlay.v' % (attr, want))
        orders = [int(v) for v in re.findall(r'Name="CompileOrder" Val="(\d+)"',
                                             body)]
        check(len(orders) == 1, "marquee_overlay.v carries a CompileOrder",
              "found %s" % orders)
        section = re.search(r"<Verilog>(.*?)</Verilog>", src.al, flags=re.S)
        if check(section is not None, "the .al has a <Verilog> section"):
            # The other sections (ADC_FILE, SDC_FILE, ...) number from 1 again,
            # so uniqueness only means anything inside this one.
            all_orders = [int(v) for v in
                          re.findall(r'Name="CompileOrder" Val="(\d+)"',
                                     section.group(1))]
            dupes = sorted({o for o in all_orders if all_orders.count(o) > 1})
            check(not dupes,
                  "CompileOrder is unique across the %d Verilog entries"
                  % len(all_orders), "duplicates: %s" % dupes)

    check("marquee_font.vh" not in src.al,
          "marquee_font.vh is NOT in the .al -- a bare function cannot compile "
          "standalone and would break the build")

    code = src.rtl
    inc = re.search(r'`include\s+"([^"]+)"', code)
    if check(inc is not None, "marquee_overlay.v includes its font table"):
        check(inc.group(1) == "marquee_font.vh",
              "the include names %s" % inc.group(1))
        check(code.index(inc.group(0)) < code.rindex("endmodule"),
              "the `include sits inside the module body, before endmodule")
        check(os.path.isfile(os.path.join(HDL, inc.group(1))),
              "%s exists next to the .v, so the include resolves without an "
              "extra search path" % inc.group(1))

    fn = re.search(r"function\s+\[\s*(\d+)\s*:\s*0\s*\]\s+marquee_glyph\s*;",
                   src.vh)
    if check(fn is not None, "the .vh defines marquee_glyph"):
        check(int(fn.group(1)) + 1 == gen.CELL,
              "marquee_glyph returns %d bits, one per glyph column"
              % (int(fn.group(1)) + 1))
    check("endfunction" in src.vh
          and not re.search(r"\bmodule\b", sim.strip_comments(src.vh)),
          "the .vh holds a bare function and no module of its own")

    reserved = []
    for tag, text in (("marquee_overlay.v", src.rtl),
                      ("marquee_font.vh", src.vh),
                      ("top_tf_hdmi_audio.v", src.top)):
        hits = sorted(declared_names(sim.strip_comments(text)) & RESERVED_WORDS)
        reserved += ["%s declares %s" % (tag, h) for h in hits]
    check(not reserved,
          "no declared identifier collides with a Verilog-2001 reserved word "
          "(TD answers that with a syntax error, not a warning)",
          "; ".join(reserved))


# ---------------------------------------------------------------------------
# 6. Negative controls
# ---------------------------------------------------------------------------
def bites(fn, src, label):
    """Run a check function against a mutated source; it must complain."""
    saved, saved_n = list(FAILURES), CHECKS[0]
    del FAILURES[:]
    CHECKS[0] = 0
    buf = io.StringIO()
    caught = False
    try:
        with contextlib.redirect_stdout(buf):
            fn(src)
        caught = bool(FAILURES)
    except SystemExit:
        caught = True            # the parser refusing to guess counts as a catch
    finally:
        del FAILURES[:]
        FAILURES.extend(saved)
        CHECKS[0] = saved_n
    expect_fail(not caught, label)


def sub_once(text, pattern, repl, what):
    new, n = re.subn(pattern, repl, text, count=1)
    if n != 1:
        raise SystemExit("control setup failed: %s not found" % what)
    return new


def check_controls(src):
    print("\n[6] negative controls: nine mutations, each must be caught")

    bites(check_sw4,
          src.clone(top=sub_once(src.top, r"assign marquee_en = sw4_v1;",
                                 "assign marquee_en = ~sw4_v1;",
                                 "marquee_en assign")),
          "C1 inverting marquee_en (banner hidden at power-up)")

    bites(check_sw4,
          src.clone(top=sub_once(src.top, r"sw_v0 <= sw\[2:0\];",
                                 "sw_v0 <= sw[3:1];", "sw_v0 sample")),
          "C2 widening the SW1-3 synchroniser to sw[3:1]")

    bites(check_geometry,
          src.clone(rtl=sub_once(src.rtl,
                                 r"localparam BAND_Y           = \(V_ACTIVE - BAND_H\) / 2;",
                                 "localparam BAND_Y           = (V_ACTIVE - BAND_H) / 2 + 1;",
                                 "BAND_Y localparam")),
          "C3 moving the band one row off centre")

    bites(check_geometry,
          src.clone(rtl=sub_once(src.rtl, r"assign cell_idx = u\[9:5\];",
                                 "assign cell_idx = u[9:4];", "cell_idx slice")),
          "C4 narrowing the cell_idx slice to u[9:4]")

    bites(check_wiring,
          src.clone(top=src.top.replace(".I_rgb (vout_data_osd)",
                                        ".I_rgb (vout_data)")
                       .replace(".O_rgb (vout_data)", ".O_rgb (vout_data_osd)")),
          "C5 swapping the marquee's input and output nets")

    mutated = sub_once(src.vh, r"24'h([0-9A-Fa-f]{6})",
                       lambda m: "24'h%06X" % ((int(m.group(1), 16) ^ 0x000001)),
                       "a glyph row in the .vh")
    bites(check_font, src.clone(vh=mutated),
          "C6 flipping one bit of one glyph row in marquee_font.vh")

    bites(check_build_files,
          src.clone(al=re.sub(r'\s*<File Path="[^"]*marquee_overlay\.v">.*?</File>',
                              "", src.al, flags=re.S)),
          "C7 dropping marquee_overlay.v from the .al")

    bites(check_build_files,
          src.clone(al=src.al.replace("</Verilog>",
                                      '    <File Path="../user_source/hdl_source/'
                                      'marquee_font.vh">\n        </File>\n'
                                      '        </Verilog>')),
          "C8 registering the bare-function .vh in the .al")

    bites(check_build_files,
          src.clone(rtl=sub_once(src.rtl, r"wire \[4:0\]  cell_idx;",
                                 "wire [4:0]  cell;", "cell_idx declaration")),
          "C9 declaring the glyph index as `cell`, a reserved word")


# ---------------------------------------------------------------------------
def main():
    print("=" * 72)
    print("marquee transcription check: RTL vs. generator vs. top vs. project")
    print("=" * 72)
    src = Sources()
    for path in (RTL, VH, TOP, AL, ADC):
        if not os.path.isfile(path):
            raise SystemExit("check_marquee_transcription: %s is missing" % path)

    check_font(src)
    check_geometry(src)
    check_wiring(src)
    check_sw4(src)
    check_build_files(src)
    check_controls(src)

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
