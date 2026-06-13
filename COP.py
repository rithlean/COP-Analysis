#!/usr/bin/env python3
"""
SPAR: Shared Point Insertion for Area Overhead Reduction
Kim et al., IEEE TCAD Vol.41 No.11, 2022

Fully autonomous flow:
  - Parses gate-level scan netlist (DC output)
  - Computes COP controllability + observability
  - Identifies CP/OP sites autonomously (no external tool needed)
  - Runs cone analysis (Rule 1 + Rule 2)
  - Runs controllability optimisation (Eq. 7/8)
  - Emits Verilog patch with shared point gates

Usage:
  python 03_spar.py netlist/scan_netlist_flat.v [DTh] [cp_budget] [op_budget]

  DTh        : difference threshold 0.0–1.0  (default 1.0, paper setting)
  cp_budget  : max CPs as fraction of scan cells (default 0.05 = 5%)
  op_budget  : max OPs as fraction of scan cells (default 0.05 = 5%)
"""

import re, json, sys, math
from collections import defaultdict, deque

# ═══════════════════════════════════════════════════════════════
#  SECTION 1 — DATA STRUCTURES
# ═══════════════════════════════════════════════════════════════

class Gate:
    __slots__ = ('gtype','name','output','inputs')
    def __init__(self, gtype, name, output, inputs):
        self.gtype   = gtype    # canonical type string
        self.name    = name
        self.output  = output   # single output net name
        self.inputs  = inputs   # list[str] of input net names

class Netlist:
    def __init__(self):
        self.gates    = {}                   # name  -> Gate
        self.net_src  = {}                   # net   -> Gate (driver)
        self.net_dst  = defaultdict(list)    # net   -> [Gate] (sinks)
        self.pis      = []                   # primary input net names
        self.pos      = []                   # primary output net names
        self.scan_ffs = []                   # scan FF output nets (PPIs)
        self.scan_din = []                   # scan FF D-input nets (PPOs)
        self.all_nets = set()

    # ── Traversal helpers ──────────────────────────────────────

    def fanout_cone(self, start_net, stop_at_ff=True):
        """BFS forward from start_net. Returns set of reachable nets."""
        visited = set()
        queue   = deque([start_net])
        while queue:
            net = queue.popleft()
            if net in visited:
                continue
            visited.add(net)
            for g in self.net_dst.get(net, []):
                if stop_at_ff and g.gtype == 'DFF':
                    continue
                if g.output not in visited:
                    queue.append(g.output)
        return visited

    def fanin_cone(self, start_net, stop_at_ff=True):
        """BFS backward from start_net. Returns set of reachable nets."""
        visited = set()
        queue   = deque([start_net])
        while queue:
            net = queue.popleft()
            if net in visited:
                continue
            visited.add(net)
            g = self.net_src.get(net)
            if g is None:
                continue
            if stop_at_ff and g.gtype == 'DFF':
                continue
            for inp in g.inputs:
                if inp not in visited:
                    queue.append(inp)
        return visited


# ═══════════════════════════════════════════════════════════════
#  SECTION 2 — GATE-LEVEL VERILOG PARSER
# ═══════════════════════════════════════════════════════════════

# Map substrings in cell names to canonical gate types.
# Add your library's cell name patterns here.
# Order matters — more specific patterns first.
CELL_TYPE_MAP = [
    # ── Flip-flops ─────────────────────────────────────────────
    ('SDFFARX',  'DFF'),   # scan FF with async reset  ← your cell
    ('SDFF',     'DFF'),
    ('DFF',      'DFF'),

    # ── Logic ──────────────────────────────────────────────────
    ('XNOR',     'XNOR'),
    ('XOR',      'XOR'),
    ('NAND',     'NAND'),
    ('NOR',      'NOR'),
    ('AND',      'AND'),
    ('OR',       'OR'),

    # ── Inverters / Buffers ────────────────────────────────────
    ('INVX',     'NOT'),   # INVX0_LVT, INVX8_LVT ...
    ('NBUFF',    'BUF'),   # NBUFFX2_LVT, NBUFFX4_LVT
    ('BUF',      'BUF'),

    # ── MUX ────────────────────────────────────────────────────
    ('MUX21',    'MUX'),   # MUX21X1_LVT — ports A1,A2,S0 → Y

    # ── Complex cells (AND-OR, OR-AND, AND-OR-INVERT) ──────────
    # These are multi-input compound gates. The COP approximation
    # treats them as AND/OR trees based on their dominant function.
    ('AOI222',   'NAND'),  # AND-OR-Invert  → approximate as NAND
    ('AOI22',    'NAND'),
    ('AOI21',    'NAND'),
    ('AO222',    'AND'),   # AND-OR         → approximate as AND
    ('AO221',    'AND'),
    ('AO22',     'AND'),
    ('AO21',     'AND'),
    ('OA221',    'OR'),    # OR-AND         → approximate as OR
    ('OA21',     'OR'),

    # ── Half-adder ─────────────────────────────────────────────
    ('HADD',     'XOR'),   # HADDX1_LVT: SO=XOR output, C1=carry
    #                        We map to XOR; carry output C1 ignored
]

# Port names that carry the output of a gate
OUTPUT_PORTS = {
    'Y',    # standard combinational output
    'Q',    # FF data output
    'QN',   # FF inverted output (used by SDFFARX1_LVT)
    'SO',   # HADDX1_LVT sum output  ← your HADD cells
    'C1',   # HADDX1_LVT carry output (secondary — handled below)
}


def canonical_type(cell_name):
    upper = cell_name.upper()
    for pattern, ctype in CELL_TYPE_MAP:
        if pattern in upper:
            return ctype
    return 'UNKNOWN'


def parse_port_list(port_str):
    """Parse .port(net) style connections into dict."""
    ports = {}
    for m in re.finditer(r'\.(\w+)\s*\(\s*(\w*)\s*\)', port_str):
        ports[m.group(1)] = m.group(2)
    return ports


def parse_verilog(filepath):
    """
    Gate-level Verilog parser for DC-generated netlists.
    Handles:
      - Single or multi-module files (takes top-level module)
      - input/output/wire declarations with optional bus widths
      - Named port connections (.port(net) style)
      - MUX-based scan flip-flops
    """
    nl = Netlist()

    with open(filepath) as f:
        raw = f.read()

    # Strip comments
    raw = re.sub(r'//[^\n]*', '', raw)
    raw = re.sub(r'/\*.*?\*/', '', raw, flags=re.DOTALL)

    # Find top-level module (last module or only module)
    modules = list(re.finditer(
        r'\bmodule\s+(\w+)\s*(?:\([^)]*\))?\s*;(.*?)endmodule',
        raw, re.DOTALL))

    if not modules:
        raise ValueError(f"No module found in {filepath}")

    # Use the last module (usually the top after flattening)
    mod_body = modules[-1].group(2)

    # ── Port declarations ──────────────────────────────────────
    def extract_signals(keyword, body):
        sigs = []
        for m in re.finditer(
                rf'\b{keyword}\b\s*(?:\[\s*\d+\s*:\s*\d+\s*\])?\s*([\w\s,]+?)\s*;',
                body):
            for sig in re.split(r'[\s,]+', m.group(1)):
                sig = sig.strip()
                if sig:
                    sigs.append(sig)
        return sigs

    nl.pis = extract_signals('input',  mod_body)
    nl.pos = extract_signals('output', mod_body)

    # ── Gate instances ─────────────────────────────────────────
    # Pattern: CellType  InstanceName  ( .port(net), ... ) ;
    inst_re = re.compile(
        r'(\w+)\s+(\w+)\s*\(([^;]*?)\)\s*;', re.DOTALL)

    for m in inst_re.finditer(mod_body):
        cell_name = m.group(1)
        inst_name = m.group(2)
        port_str  = m.group(3)

        # Skip keywords that look like instances
        if cell_name in ('module','input','output','wire',
                         'reg','assign','always','if','else'):
            continue

        ctype = canonical_type(cell_name)
        if ctype == 'UNKNOWN':
            continue

        ports = parse_port_list(port_str)
        # Normalise MUX21X1_LVT ports to canonical D0, D1, SEL naming
        if 'S0' in ports:
            ports['SEL'] = ports.pop('S0')
        if 'A1' in ports and 'A2' in ports and 'SEL' in ports:
            ports['D0'] = ports.pop('A1')
            ports['D1'] = ports.pop('A2')

        if not ports:
            continue

        # Find output net
        out_net = None
        for op in OUTPUT_PORTS:
            if op in ports and ports[op]:
                out_net = ports[op]
                break

        if out_net is None:
            # Fallback: last port is often output
            vals = list(ports.values())
            if vals:
                out_net = vals[-1]
            else:
                continue

        inp_nets = [v for k, v in ports.items()
                    if k not in OUTPUT_PORTS and v and v != out_net]

        g = Gate(ctype, inst_name, out_net, inp_nets)
        nl.gates[inst_name] = g
        nl.net_src[out_net] = g
        for inp in inp_nets:
            nl.net_dst[inp].append(g)
        nl.all_nets.add(out_net)
        nl.all_nets.update(inp_nets)

        # HADD has two outputs: SO (sum, already captured above)
        # and C1 (carry). Register C1 as a separate AND gate.
        if ctype == 'XOR' and 'C1' in ports and ports.get('C1'):
            carry_net  = ports['C1']
            carry_gate = Gate('AND',
                              inst_name + '_carry',
                              carry_net,
                              inp_nets)
            nl.gates[inst_name + '_carry'] = carry_gate
            nl.net_src[carry_net]          = carry_gate
            for inp in inp_nets:
                nl.net_dst[inp].append(carry_gate)
            nl.all_nets.add(carry_net)

        # Track scan FFs
        if ctype == 'DFF':
            nl.scan_ffs.append(out_net)        # Q output = PPI
            if inp_nets:
                nl.scan_din.append(inp_nets[0]) # D input  = PPO

    print(f"  Parsed: {len(nl.gates)} gates, "
          f"{len(nl.pis)} PIs, "
          f"{len(nl.scan_ffs)} scan FFs")
    return nl


# ═══════════════════════════════════════════════════════════════
#  SECTION 3 — COP TESTABILITY (Paper Section II-A)
# ═══════════════════════════════════════════════════════════════

def compute_cop(nl):
    """
    COP algorithm.
    CC[net] = probability net = logic-1 under uniform random inputs.
    CO[net] = probability a fault on net propagates to a PO or scan FF D-pin.

    PIs and scan FF outputs (PPIs) initialised to 0.5.
    POs and scan FF D-inputs (PPOs) have observability 1.0.
    """

    CC = {}
    CO = {}

    # ── Initialise ────────────────────────────────────────────
    for pi  in nl.pis:      CC[pi]  = 0.5
    for ppi in nl.scan_ffs: CC[ppi] = 0.5   # FF output treated as random
    for po  in nl.pos:      CO[po]  = 1.0
    for ppo in nl.scan_din: CO[ppo] = 1.0   # FF D-input is observable

    # ── Forward pass: compute CC via topological BFS ──────────
    # Repeat until stable (handles reconvergent fanouts approximately)
    MAX_ITER = 500
    for _ in range(MAX_ITER):
        changed = False
        for g in nl.gates.values():
            if not all(i in CC for i in g.inputs):
                continue
            ins = [CC[i] for i in g.inputs]
            t   = g.gtype

            if   t == 'AND':
                cc = 1.0
                for v in ins: cc *= v
            elif t == 'OR':
                cc = 1.0
                for v in ins: cc *= (1.0 - v)
                cc = 1.0 - cc
            elif t == 'NAND':
                cc = 1.0
                for v in ins: cc *= v
                cc = 1.0 - cc
            elif t == 'NOR':
                cc = 1.0
                for v in ins: cc *= (1.0 - v)
            elif t == 'NOT':
                cc = 1.0 - ins[0]
            elif t == 'BUF':
                cc = ins[0]
            elif t == 'XOR':
                cc = ins[0]
                for v in ins[1:]:
                    cc = cc * (1.0 - v) + (1.0 - cc) * v
            elif t == 'XNOR':
                cc = ins[0]
                for v in ins[1:]:
                    cc = cc * (1.0 - v) + (1.0 - cc) * v
                cc = 1.0 - cc
            elif t == 'MUX':
                # D0, D1, SEL  →  out = SEL?D1:D0
                if len(ins) >= 3:
                    d0, d1, sel = ins[0], ins[1], ins[2]
                    cc = sel * d1 + (1.0 - sel) * d0
                else:
                    cc = 0.5
            elif t == 'DFF':
                cc = 0.5    # output already seeded; skip
            else:
                cc = 0.5

            cc = max(1e-9, min(1.0 - 1e-9, cc))
            if abs(CC.get(g.output, -1) - cc) > 1e-10:
                CC[g.output] = cc
                changed = True

        if not changed:
            break

    # ── Backward pass: compute CO ─────────────────────────────
    for _ in range(MAX_ITER):
        changed = False
        for g in nl.gates.values():
            if g.output not in CO:
                continue
            co_out = CO[g.output]
            ins_cc = [CC.get(i, 0.5) for i in g.inputs]
            t      = g.gtype

            for idx, inp in enumerate(g.inputs):
                others = [ins_cc[j] for j in range(len(ins_cc))
                          if j != idx]

                if   t in ('AND', 'NAND'):
                    # Sensitise: all other inputs must be 1
                    sens = 1.0
                    for v in others: sens *= v
                elif t in ('OR', 'NOR'):
                    # Sensitise: all other inputs must be 0
                    sens = 1.0
                    for v in others: sens *= (1.0 - v)
                elif t in ('NOT', 'BUF'):
                    sens = 1.0
                elif t in ('XOR', 'XNOR'):
                    # XOR always sensitisable (parity gate)
                    sens = 1.0
                elif t == 'MUX':
                    if len(ins_cc) >= 3:
                        sel = ins_cc[2]
                        if   idx == 0: sens = 1.0 - sel
                        elif idx == 1: sens = sel
                        else:          sens = abs(ins_cc[0] - ins_cc[1])
                    else:
                        sens = 0.5
                elif t == 'DFF':
                    sens = 1.0   # D-input is observable
                else:
                    sens = 0.5

                new_co = co_out * sens
                if new_co > CO.get(inp, 0.0) + 1e-12:
                    CO[inp] = new_co
                    changed = True

        if not changed:
            break

    return CC, CO


# ═══════════════════════════════════════════════════════════════
#  SECTION 4 — AUTONOMOUS CP/OP SITE IDENTIFICATION
# ═══════════════════════════════════════════════════════════════

def identify_cp_op_sites(nl, CC, CO, cp_budget_frac=0.05,
                          op_budget_frac=0.05):
    """
    Autonomous identification of CP and OP sites using COP metrics.
    Mirrors the conventional TPI step in the paper's Fig. 4.

    CP sites: internal nets with low controllability
              (hard-to-1 → OR-type, hard-to-0 → AND-type)
    OP sites: internal nets with low observability

    Budget: paper limits each to 5% of scan cell count.
    Returns:
      cp_list: [(net, 'AND'|'OR'), ...]
      op_list: [net, ...]
    """
    n_scan   = max(len(nl.scan_ffs), 1)
    cp_limit = max(1, int(n_scan * cp_budget_frac))
    op_limit = max(1, int(n_scan * op_budget_frac))

    # Only consider internal combinational nets (not PIs/POs/FF outputs)
    excluded = set(nl.pis) | set(nl.pos) | \
               set(nl.scan_ffs) | set(nl.scan_din)

    internal_nets = [n for n in nl.all_nets if n not in excluded]

    # ── CP candidates: sort by distance from 0.5 (most biased first) ──
    def cp_score(net):
        cc = CC.get(net, 0.5)
        return abs(cc - 0.5)   # higher = more biased = harder to control

    cp_candidates = sorted(internal_nets,
                           key=cp_score, reverse=True)

    cp_list = []
    for net in cp_candidates:
        if len(cp_list) >= cp_limit:
            break
        cc = CC.get(net, 0.5)
        # Skip nets that are already near-0.5 (not truly hard-to-control)
        if abs(cc - 0.5) < 0.05:
            continue
        cp_type = 'OR' if cc < 0.5 else 'AND'
        cp_list.append((net, cp_type))

    # ── OP candidates: sort by ascending observability ────────
    op_candidates = sorted(internal_nets,
                           key=lambda n: CO.get(n, 0.0))

    op_list = []
    for net in op_candidates:
        if len(op_list) >= op_limit:
            break
        co = CO.get(net, 0.0)
        # Skip nets that are already easily observable
        if co > 0.1:
            break
        op_list.append(net)

    print(f"  CP sites identified : {len(cp_list)} "
          f"(budget {cp_limit}, {cp_budget_frac*100:.0f}% of "
          f"{n_scan} scan FFs)")
    print(f"  OP sites identified : {len(op_list)} "
          f"(budget {op_limit})")
    return cp_list, op_list


# ═══════════════════════════════════════════════════════════════
#  SECTION 5 — THRESHOLD COMPUTATION (Paper Eq. 5 & 6)
# ═══════════════════════════════════════════════════════════════

def compute_thresholds(cp_list, op_list, CC, CO):
    """
    CCTh = max(avg CC of OR-CPs,  avg (1-CC) of AND-CPs)   Eq. 5
    COTh = avg CO of OPs                                    Eq. 6
    """
    or_cc  = [CC.get(n, 0.5)       for n, t in cp_list if t == 'OR']
    and_cc = [1.0 - CC.get(n, 0.5) for n, t in cp_list if t == 'AND']

    avg_or  = sum(or_cc)  / max(len(or_cc),  1)
    avg_and = sum(and_cc) / max(len(and_cc), 1)
    CCTh    = max(avg_or, avg_and)

    op_co  = [CO.get(n, 0.0) for n in op_list]
    COTh   = sum(op_co) / max(len(op_co), 1)

    return CCTh, COTh


# ═══════════════════════════════════════════════════════════════
#  SECTION 6 — CANDIDATE IDENTIFICATION (Paper Section IV)
# ═══════════════════════════════════════════════════════════════

def identify_shared_candidates(cp_list, op_list, CC, CO,
                                CCTh, COTh):
    """
    EO-CPs : CP lines where CO > COTh  (easily observable)
    EC-OPs : OP lines where CCTh <= CC <= 1-CCTh  (easily controllable)
    """
    eo_cps = [(net, t) for net, t in cp_list
              if CO.get(net, 0.0) > COTh]

    ec_ops = [net for net in op_list
              if CCTh <= CC.get(net, 0.5) <= 1.0 - CCTh]

    print(f"  EO-CPs : {len(eo_cps)} / {len(cp_list)}")
    print(f"  EC-OPs : {len(ec_ops)} / {len(op_list)}")
    return eo_cps, ec_ops


# ═══════════════════════════════════════════════════════════════
#  SECTION 7 — CONE ANALYSIS (Paper Section V)
# ═══════════════════════════════════════════════════════════════

def check_rule1(nl, eo_cp_net, ec_op_net):
    """
    Rule 1: EC-OP must NOT be in the fanout cone of EO-CP.
    Prevents combinational loop generation.
    Verified after every insertion (topology changes with each SP).
    """
    fanout = nl.fanout_cone(eo_cp_net)
    return ec_op_net not in fanout      # True = safe to pair


def check_rule2(nl, eo_cp_net, ec_op_net, cp_type, CC,
                direct):
    """
    Rule 2: injecting the control value onto ec_op_net must not
    block fault propagation through the blocked cone of eo_cp_net.

    Returns True if this control scheme is safe.
    """
    # Determine which value drives the shared point
    # OR-type CP: needs 1 on control line to activate  (Fig. 3c/d)
    # AND-type CP: needs 0 on control line to activate (Fig. 3a/b)
    if cp_type == 'OR':
        ctrl_val = 1.0 if direct else 0.0
    else:
        ctrl_val = 0.0 if direct else 1.0

    # Temporarily propagate ctrl_val forward from ec_op_net
    temp_CC = dict(CC)
    temp_CC[ec_op_net] = ctrl_val

    queue   = deque(nl.net_dst.get(ec_op_net, []))
    visited = set()

    while queue:
        g = queue.popleft()
        if g.name in visited:
            continue
        visited.add(g.name)
        if g.gtype == 'DFF':
            continue

        ins = [temp_CC.get(i, CC.get(i, 0.5)) for i in g.inputs]
        t   = g.gtype

        new_val = None
        if   t in ('AND', 'NAND'):
            if all(v > 0.99 for v in ins):
                new_val = 0.0 if t == 'NAND' else 1.0
            elif any(v < 0.01 for v in ins):
                new_val = 1.0 if t == 'NAND' else 0.0
        elif t in ('OR', 'NOR'):
            if any(v > 0.99 for v in ins):
                new_val = 0.0 if t == 'NOR' else 1.0
            elif all(v < 0.01 for v in ins):
                new_val = 1.0 if t == 'NOR' else 0.0
        elif t == 'NOT':
            new_val = 1.0 - ins[0] if ins else None
        elif t == 'BUF':
            new_val = ins[0] if ins else None

        if new_val is not None:
            temp_CC[g.output] = new_val
            for ng in nl.net_dst.get(g.output, []):
                if ng.name not in visited:
                    queue.append(ng)

    # Check observability of eo_cp_net's direct fanout gates
    # under the modified controllabilities
    direct_sinks = nl.net_dst.get(eo_cp_net, [])
    if not direct_sinks:
        return True   # no fanout — no blocking possible

    any_unblocked = False
    for sink_gate in direct_sinks:
        if sink_gate.gtype == 'DFF':
            any_unblocked = True
            continue
        # Compute sensitisation probability through this gate
        other_ins = [temp_CC.get(i, CC.get(i, 0.5))
                     for i in sink_gate.inputs
                     if i != eo_cp_net]
        t = sink_gate.gtype

        if t in ('AND', 'NAND'):
            # All others must be 1 to sensitise
            sens = 1.0
            for v in other_ins: sens *= v
        elif t in ('OR', 'NOR'):
            # All others must be 0 to sensitise
            sens = 1.0
            for v in other_ins: sens *= (1.0 - v)
        elif t in ('NOT', 'BUF', 'XOR', 'XNOR'):
            sens = 1.0
        else:
            sens = 0.5

        if sens > 1e-6:
            any_unblocked = True
            break

    return any_unblocked     # True = at least one path unblocked


# ═══════════════════════════════════════════════════════════════
#  SECTION 8 — CONTROLLABILITY OPTIMISATION (Paper Section VI)
# ═══════════════════════════════════════════════════════════════

def compute_cc_req(nl, eo_cp_net, cp_type):
    """
    Eq. 7  OR-type  : CCreq = Bx / (bx + Bx + Fx)
    Eq. 8  AND-type : CCreq = (Bx + Fx) / (bx + Bx + Fx)

    bx = faults blocked when eo_cp_net = 1
         (gates in direct fanout that are dominated by 1)
    Bx = faults blocked when eo_cp_net = 0
         (gates in direct fanout that are dominated by 0)
    Fx = faults propagating through eo_cp_net (fanout cone size)

    Approximation: use fanout cone sizes as fault count proxies,
    consistent with the paper's notation in Section VI.
    """
    fanout_total = len(nl.fanout_cone(eo_cp_net))
    direct_sinks = nl.net_dst.get(eo_cp_net, [])

    bx = 0   # blocked when net=1  (OR/NOR dominated)
    Bx = 0   # blocked when net=0  (AND/NAND dominated)

    for g in direct_sinks:
        if g.gtype == 'DFF':
            continue
        cone_size = len(nl.fanout_cone(g.output))
        if g.gtype in ('OR', 'NOR'):
            # eo_cp=1 dominates OR → blocks propagation of other faults
            bx += cone_size
        elif g.gtype in ('AND', 'NAND'):
            # eo_cp=0 dominates AND → blocks propagation
            Bx += cone_size

    Fx    = max(fanout_total - bx - Bx, 0)
    total = bx + Bx + Fx

    if total == 0:
        return 0.5

    if cp_type == 'OR':
        return Bx / total                   # Eq. 7
    else:
        return (Bx + Fx) / total            # Eq. 8


def select_best_ec_op(candidate_nets, cc_req, CC, DTh=1.0):
    """
    Select EC-OP whose controllability (direct or inverted)
    is closest to cc_req, within DTh tolerance (Eq. 9).
    Returns (net, use_inverted) or (None, None).
    """
    best_net  = None
    best_inv  = False
    best_diff = float('inf')

    for net in candidate_nets:
        cc      = CC.get(net, 0.5)
        d_dir   = abs(cc_req - cc)
        d_inv   = abs(cc_req - (1.0 - cc))

        if d_dir <= d_inv and d_dir < best_diff:
            best_diff, best_net, best_inv = d_dir, net, False
        elif d_inv < d_dir and d_inv < best_diff:
            best_diff, best_net, best_inv = d_inv, net, True

    if best_net is None or best_diff >= DTh:
        return None, None

    return best_net, best_inv


# ═══════════════════════════════════════════════════════════════
#  SECTION 9 — MAIN SPAR FLOW (Paper Fig. 4)
# ═══════════════════════════════════════════════════════════════

def run_spar(netlist_path,
             DTh=1.0,
             cp_budget=0.05,
             op_budget=0.05):

    print("\n" + "="*55)
    print(" SPAR: Shared Point Insertion for Area Overhead Reduction")
    print("="*55)

    # ── Step 1: Parse netlist ──────────────────────────────────
    print("\n[1/7] Parsing netlist ...")
    nl = parse_verilog(netlist_path)

    # ── Step 2: COP analysis ───────────────────────────────────
    print("\n[2/7] COP testability analysis ...")
    CC, CO = compute_cop(nl)
    print(f"  CC computed for {len(CC)} nets")
    print(f"  CO computed for {len(CO)} nets")

    # ── Step 3: CP/OP site identification ─────────────────────
    print("\n[3/7] Autonomous CP/OP site identification ...")
    cp_list, op_list = identify_cp_op_sites(
        nl, CC, CO, cp_budget, op_budget)

    # ── Step 4: Threshold computation ─────────────────────────
    print("\n[4/7] Computing EO-CP / EC-OP thresholds ...")
    CCTh, COTh = compute_thresholds(cp_list, op_list, CC, CO)
    print(f"  CCTh = {CCTh:.6f}")
    print(f"  COTh = {COTh:.6f}")

    # ── Step 5: Shared point candidates ───────────────────────
    print("\n[5/7] Identifying shared point candidates ...")
    eo_cps, ec_ops = identify_shared_candidates(
        cp_list, op_list, CC, CO, CCTh, COTh)

    # ── Step 6: Pairing ───────────────────────────────────────
    print("\n[6/7] Pairing (cone analysis + ctrl optimisation) ...")

    shared_points = []
    used_ec_ops   = set()
    conventional_cps  = []   # EO-CPs that could not find a pair
    conventional_ops  = []   # EC-OPs that were not consumed

    for eo_cp_net, cp_type in eo_cps:
        cc_req = compute_cc_req(nl, eo_cp_net, cp_type)

        # Build valid candidate list for this EO-CP
        valid = []
        for ec_op_net in ec_ops:
            if ec_op_net in used_ec_ops:
                continue

            # Rule 1: no combinational loop
            if not check_rule1(nl, eo_cp_net, ec_op_net):
                continue

            # Rule 2: check both control schemes
            ok_direct   = check_rule2(nl, eo_cp_net, ec_op_net,
                                      cp_type, CC, direct=True)
            ok_inverted = check_rule2(nl, eo_cp_net, ec_op_net,
                                      cp_type, CC, direct=False)

            if ok_direct or ok_inverted:
                valid.append((ec_op_net, ok_direct, ok_inverted))

        if not valid:
            conventional_cps.append((eo_cp_net, cp_type))
            continue

        # Filter to nets with at least one valid scheme
        valid_nets = [n for n, od, oi in valid]

        best_net, use_inv = select_best_ec_op(
            valid_nets, cc_req, CC, DTh)

        if best_net is None:
            conventional_cps.append((eo_cp_net, cp_type))
            continue

        # Determine control scheme validity for chosen net
        ok_d = next((od for n, od, oi in valid if n == best_net), False)
        ok_i = next((oi for n, od, oi in valid if n == best_net), False)

        # If use_inv conflicts with rule2, override
        if use_inv and not ok_i:
            use_inv = False
        elif not use_inv and not ok_d:
            use_inv = True

        # Shared point type (paper Fig. 3):
        #   AND + direct   = Type 1
        #   AND + inverted = Type 2
        #   OR  + direct   = Type 3
        #   OR  + inverted = Type 4
        sp_type = (1 if cp_type == 'AND' and not use_inv else
                   2 if cp_type == 'AND' and use_inv     else
                   3 if cp_type == 'OR'  and not use_inv else 4)

        cc_actual = (1.0 - CC.get(best_net, 0.5)) \
                    if use_inv else CC.get(best_net, 0.5)

        sp = {
            'eo_cp_net' : eo_cp_net,
            'ec_op_net' : best_net,
            'cp_type'   : cp_type,
            'sp_type'   : sp_type,
            'inverted'  : use_inv,
            'cc_req'    : round(cc_req,    6),
            'cc_actual' : round(cc_actual, 6),
            'cc_diff'   : round(abs(cc_req - cc_actual), 6),
        }
        shared_points.append(sp)
        used_ec_ops.add(best_net)

        # Update topology so Rule 1 is accurate for subsequent pairs
        bridge = Gate('BUF',
                      f'__spar_bridge_{len(shared_points)}',
                      eo_cp_net,
                      [best_net])
        nl.net_dst[best_net].append(bridge)

    # Remaining OPs not consumed become conventional OPs
    conventional_ops = [n for n in op_list if n not in used_ec_ops]

    # ── Step 7: Summary ───────────────────────────────────────
    print("\n[7/7] Results summary")
    print(f"  Shared points inserted   : {len(shared_points)}")
    print(f"  Conventional CPs remain  : {len(conventional_cps)}")
    print(f"  Conventional OPs remain  : {len(conventional_ops)}")

    sp_ratio = len(shared_points) / max(len(eo_cps), 1) * 100
    print(f"  SP ratio (of EO-CPs)     : {sp_ratio:.1f}%")

    area_saving_est = len(shared_points) / \
                      max(len(cp_list) + len(op_list), 1) * 100
    print(f"  Estimated area reduction : ~{area_saving_est:.1f}% "
          f"(of conventional TP logic)")

    return shared_points, conventional_cps, conventional_ops, nl


# ═══════════════════════════════════════════════════════════════
#  SECTION 10 — VERILOG PATCH GENERATOR
# ═══════════════════════════════════════════════════════════════

# Paper Fig. 3 — four shared point structures
# TPEnable = 1 during test, 0 during functional operation
# {eo_cp}_new replaces all downstream uses of {eo_cp}

SP_GATE_TEMPLATES = {
    # Type 1: AND-type, direct control
    # EO-CP line needs 0; EC-OP provides 0 directly
    # Structure: AND(EO-CP, NOT(AND(EC-OP, TPEnable)))
    1 : """\
  // Shared Point Type 1 — AND direct
  // EO-CP={ecp}  EC-OP={eop}  CCreq={cc_req}  CCactual={cc_act}
  wire _sp{i}_ctrl, _sp{i}_out;
  and  _sp{i}_en  (_sp{i}_ctrl, {eop}, TPEnable);
  and  _sp{i}_gate(_sp{i}_out,  {ecp}, _sp{i}_ctrl);
  // replace downstream uses of {ecp} with _sp{i}_out
""",
    # Type 2: AND-type, inverted control
    # Structure: AND(EO-CP, NAND(EC-OP, TPEnable))
    2 : """\
  // Shared Point Type 2 — AND inverted
  // EO-CP={ecp}  EC-OP={eop}  CCreq={cc_req}  CCactual={cc_act}
  wire _sp{i}_ctrl, _sp{i}_out;
  nand _sp{i}_en  (_sp{i}_ctrl, {eop}, TPEnable);
  and  _sp{i}_gate(_sp{i}_out,  {ecp}, _sp{i}_ctrl);
  // replace downstream uses of {ecp} with _sp{i}_out
""",
    # Type 3: OR-type, direct control
    # Structure: OR(EO-CP, AND(EC-OP, TPEnable))
    3 : """\
  // Shared Point Type 3 — OR direct
  // EO-CP={ecp}  EC-OP={eop}  CCreq={cc_req}  CCactual={cc_act}
  wire _sp{i}_ctrl, _sp{i}_out;
  and  _sp{i}_en  (_sp{i}_ctrl, {eop}, TPEnable);
  or   _sp{i}_gate(_sp{i}_out,  {ecp}, _sp{i}_ctrl);
  // replace downstream uses of {ecp} with _sp{i}_out
""",
    # Type 4: OR-type, inverted control
    # Structure: OR(EO-CP, NAND(EC-OP, TPEnable))
    4 : """\
  // Shared Point Type 4 — OR inverted
  // EO-CP={ecp}  EC-OP={eop}  CCreq={cc_req}  CCactual={cc_act}
  wire _sp{i}_ctrl, _sp{i}_out;
  nand _sp{i}_en  (_sp{i}_ctrl, {eop}, TPEnable);
  or   _sp{i}_gate(_sp{i}_out,  {ecp}, _sp{i}_ctrl);
  // replace downstream uses of {ecp} with _sp{i}_out
""",
}


def generate_verilog_patch(shared_points, conventional_cps,
                            conventional_ops, output_path):
    """
    Emit a Verilog snippet containing all shared point gates.
    Intended to be appended inside the top module of scan_netlist.v
    before the endmodule keyword.
    """
    lines = [
        "// ─────────────────────────────────────────────────────",
        "// SPAR shared-point logic — auto-generated",
        "// Insert this block inside the top module,",
        "// before endmodule.",
        "// TPEnable: 1 = test mode, 0 = functional mode",
        "// ─────────────────────────────────────────────────────",
        "",
        "input TPEnable;   // add to module port list",
        "",
    ]

    for i, sp in enumerate(shared_points):
        tmpl = SP_GATE_TEMPLATES[sp['sp_type']]
        lines.append(tmpl.format(
            i       = i,
            ecp     = sp['eo_cp_net'],
            eop     = sp['ec_op_net'],
            cc_req  = sp['cc_req'],
            cc_act  = sp['cc_actual'],
        ))

    # Conventional CPs still need dedicated drivers
    if conventional_cps:
        lines.append("// ── Conventional CPs (no EC-OP pair found) ──")
        for net, cp_type in conventional_cps:
            gate = 'or' if cp_type == 'OR' else 'and'
            lines.append(
                f"  {gate} _conv_cp_{net} "
                f"(_conv_{net}_out, {net}, TPEnable);  "
                f"// {cp_type}-type CP")
        lines.append("")

    with open(output_path, 'w') as f:
        f.write('\n'.join(lines))

    print(f"[+] Verilog patch → {output_path}")


# ═══════════════════════════════════════════════════════════════
#  SECTION 11 — JSON REPORT
# ═══════════════════════════════════════════════════════════════

def write_json_report(shared_points, conventional_cps,
                      conventional_ops, output_path):
    report = {
        'summary' : {
            'shared_points'     : len(shared_points),
            'conventional_cps'  : len(conventional_cps),
            'conventional_ops'  : len(conventional_ops),
        },
        'shared_points'    : shared_points,
        'conventional_cps' : [{'net': n, 'type': t}
                               for n, t in conventional_cps],
        'conventional_ops' : conventional_ops,
    }
    with open(output_path, 'w') as f:
        json.dump(report, f, indent=2)
    print(f"[+] JSON report    → {output_path}")


# ═══════════════════════════════════════════════════════════════
#  ENTRY POINT
# ═══════════════════════════════════════════════════════════════

if __name__ == '__main__':
    netlist_path = (sys.argv[1]
                    if len(sys.argv) > 1
                    else "netlist/scan_netlist_flat.v")
    DTh          = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0
    cp_bud       = float(sys.argv[3]) if len(sys.argv) > 3 else 0.05
    op_bud       = float(sys.argv[4]) if len(sys.argv) > 4 else 0.05

    sps, conv_cps, conv_ops, nl = run_spar(
        netlist_path, DTh, cp_bud, op_bud)

    write_json_report(sps, conv_cps, conv_ops,
                      "reports/shared_points.json")
    generate_verilog_patch(sps, conv_cps, conv_ops,
                           "reports/spar_patch.v")
    print("\nDone. Next step: 04_resynth.tcl")