"""
COP (Controllability/Observability Probability) implementation
for SAED32nm gate-level Verilog netlists synthesized by Synopsys DC.

Based on: Brglez et al., "Applications of testability analysis:
From ATPG to critical delay path tracing," ITC 1984.

Usage:
    python cop.py <netlist.v> [--tp_file <tp_analysis.txt>]
"""

import re
import sys
from collections import defaultdict, deque

# ---------------------------------------------------------------------------
# HELPERS
# ---------------------------------------------------------------------------

def prod(vals):
    r = 1.0
    for v in vals:
        r *= v
    return r

def clamp(v):
    return max(0.0, min(1.0, v))

# ---------------------------------------------------------------------------
# CELL TABLE  ->  (input_ports, output_ports)
# ---------------------------------------------------------------------------

CELL_OUTPUT_PORTS = {
    'AND2':   (['A1','A2'],                        ['Y']),
    'AND3':   (['A1','A2','A3'],                   ['Y']),
    'AND4':   (['A1','A2','A3','A4'],              ['Y']),
    'OR2':    (['A1','A2'],                        ['Y']),
    'OR3':    (['A1','A2','A3'],                   ['Y']),
    'OR4':    (['A1','A2','A3','A4'],              ['Y']),
    'NAND2':  (['A1','A2'],                        ['Y']),
    'NAND3':  (['A1','A2','A3'],                   ['Y']),
    'NAND4':  (['A1','A2','A3','A4'],              ['Y']),
    'NOR2':   (['A1','A2'],                        ['Y']),
    'NOR3':   (['A1','A2','A3'],                   ['Y']),
    'NOR4':   (['A1','A2','A3','A4'],              ['Y']),
    'INV':    (['A'],                              ['Y']),
    'BUF':    (['A'],                              ['Y']),
    'NBUFF':  (['A'],                              ['Y']),
    'IBUFF':  (['A'],                              ['Y']),
    'AOI21':  (['A1','A2','A3'],                   ['Y']),
    'AOI22':  (['A1','A2','A3','A4'],              ['Y']),
    'OAI21':  (['A1','A2','A3'],                   ['Y']),
    'OAI22':  (['A1','A2','A3','A4'],              ['Y']),
    'OAI221': (['A1','A2','A3','A4','A5'],         ['Y']),
    'OAI222': (['A1','A2','A3','A4','A5','A6'],    ['Y']),
    'AOI221': (['A1','A2','A3','A4','A5'],         ['Y']),
    'AOI222': (['A1','A2','A3','A4','A5','A6'],    ['Y']),
    'AO21':   (['A1','A2','A3'],                   ['Y']),
    'AO22':   (['A1','A2','A3','A4'],              ['Y']),
    'AO221':  (['A1','A2','A3','A4','A5'],         ['Y']),
    'AO222':  (['A1','A2','A3','A4','A5','A6'],    ['Y']),
    'OA21':   (['A1','A2','A3'],                   ['Y']),
    'OA22':   (['A1','A2','A3','A4'],              ['Y']),
    'OA221':  (['A1','A2','A3','A4','A5'],         ['Y']),
    'OA222':  (['A1','A2','A3','A4','A5','A6'],    ['Y']),
    'MUX21':  (['A1','A2','S0'],                   ['Y']),
    'MUX41':  (['A1','A2','A3','A4','S0','S1'],    ['Y']),
    'HADD':   (['A0','B0'],                        ['SO','CO']),
    'FADD':   (['A','B','CI'],                     ['S','CO']),
    # Flip-flops — only D is the combinational input for COP purposes
    'SDFFX1': (['D'],                              ['Q','QN']),
    'SDFFX2': (['D'],                              ['Q','QN']),
    'DFFX1':  (['D'],                              ['Q','QN']),
}

unknown_cells = set()


def get_cell_base(cell_name):
    """
    Map SAED32 cell name to the base key used in CELL_OUTPUT_PORTS.
    SDFFX2_LVT -> SDFFX2,  DFFX1_LVT -> DFFX1,  NAND2X0_LVT -> NAND2
    Strategy: strip _LVT, then check if it is a known FF name before
    stripping drive strength (X<n>), because FFs carry X<n> as part of
    the cell family name.
    """
    name = re.sub(r'_LVT$', '', cell_name)
    # If the name after removing _LVT is already a known cell, return it
    if name in CELL_OUTPUT_PORTS:
        return name
    # Strip trailing drive strength  (X0 / X1 / X2 ...)
    stripped = re.sub(r'X\d+$', '', name)
    if stripped in CELL_OUTPUT_PORTS:
        return stripped
    # Return stripped anyway so unknown_cells tracking works
    return stripped


# ---------------------------------------------------------------------------
# NETLIST PARSER
# ---------------------------------------------------------------------------

def parse_verilog(filename):
    """
    Parse a flat gate-level Verilog netlist produced by Synopsys DC.
    Returns:
        ports_in   : set of input port net names
        ports_out  : set of output port net names
        instances  : list of dicts  {cell, inst, conn{port->net}}
        assigns    : list of (lhs_net, rhs_net)
        pin_to_net : dict  "INST/PORT" -> net_name
    """
    with open(filename, 'r') as f:
        text = f.read()

    # Join backslash-continued lines (DC wraps long lines)
    text = re.sub(r'\\\n\s*', ' ', text)
    # Remove block comments
    text = re.sub(r'/\*.*?\*/', ' ', text, flags=re.DOTALL)
    # Remove line comments
    text = re.sub(r'//[^\n]*', '', text)

    # ---- port declarations ----
    ports_in, ports_out = set(), set()
    for m in re.finditer(r'\binput\b([^;]+);', text):
        for net in re.split(r'[,\s]+', m.group(1)):
            net = net.strip()
            if net:
                ports_in.add(net)
    for m in re.finditer(r'\boutput\b([^;]+);', text):
        for net in re.split(r'[,\s]+', m.group(1)):
            net = net.strip()
            if net:
                ports_out.add(net)

    # ---- assign statements ----
    assigns = []
    for m in re.finditer(r'\bassign\s+(\S+)\s*=\s*(\S+)\s*;', text):
        assigns.append((m.group(1), m.group(2)))

    # ---- gate instances ----
    # Pattern:  CELLTYPE  INSTNAME  ( .PORT(NET), ... );
    instances  = []
    pin_to_net = {}   # "INST/PORT" -> net_name

    inst_re = re.compile(
        r'(\w+_LVT)\s+(\\?[\w/\[\]]+)\s*\(([^;]+)\);', re.DOTALL)
    port_re = re.compile(r'\.(\w+)\s*\(([^)]*)\)')

    for m in inst_re.finditer(text):
        cell     = m.group(1)
        inst     = m.group(2)
        conn_str = m.group(3)
        conn     = {}
        for pm in port_re.finditer(conn_str):
            port = pm.group(1)
            net  = pm.group(2).strip()
            if net:
                conn[port] = net
                pin_to_net['{}/{}'.format(inst, port)] = net
        instances.append({'cell': cell, 'inst': inst, 'conn': conn})

    return ports_in, ports_out, instances, assigns, pin_to_net


# ---------------------------------------------------------------------------
# TP FILE PARSER
# ---------------------------------------------------------------------------

def parse_tp_file(tp_file):
    """
    Parse TetraMAX tp_analysis.txt.
    Handles backslash-continued lines.
    Returns:
        cps : list of (pin_ref, type)   type in {'control_0', 'control_1'}
        ops : list of pin_ref
    """
    with open(tp_file, 'r') as f:
        raw = f.read()

    # Join continuation lines before splitting
    raw = re.sub(r'\\\n\s*', ' ', raw)

    cps, ops = [], []
    for line in raw.splitlines():
        line = line.strip()
        if not line or line.startswith('#'):
            continue
        m = re.match(
            r'set_test_point_element\s*\{([^}]+)\}\s*-type\s+(\S+)', line)
        if m:
            refs    = [r.strip() for r in m.group(1).split() if r.strip()]
            tp_type = m.group(2)
            if tp_type in ('control_0', 'control_1'):
                for r in refs:
                    cps.append((r, tp_type))
            elif tp_type == 'observe':
                for r in refs:
                    ops.append(r)
    return cps, ops


# ---------------------------------------------------------------------------
# PIN-REF RESOLVER
# TetraMAX names test points as "INST/Y" (instance / output-port).
# We need to map this to the actual net name driven by that pin.
# ---------------------------------------------------------------------------

def make_lookup(pin_to_net, cc1_vals):
    """Return a closure that resolves a TP pin-ref to a net name."""
    def lookup(ref):
        # 1. Already a direct net name
        if ref in cc1_vals:
            return ref
        # 2. Exact key in pin_to_net
        if ref in pin_to_net:
            return pin_to_net[ref]
        # 3. Leading backslash variants
        alt = ref.lstrip('\\')
        if alt in pin_to_net:
            return pin_to_net[alt]
        backslashed = '\\' + ref
        if backslashed in pin_to_net:
            return pin_to_net[backslashed]
        return None
    return lookup


# ---------------------------------------------------------------------------
# COP FORWARD PASS — compute CC1 for every net
# ---------------------------------------------------------------------------

def compute_cc1(base, input_cc1):
    """
    Return dict {output_port: cc1_value} for a gate.
    input_cc1 : dict {input_port: cc1_value}
    """
    def c1(p): return input_cc1.get(p, 0.5)
    def c0(p): return 1.0 - input_cc1.get(p, 0.5)
    R = {}

    if base in ('AND2', 'AND3', 'AND4'):
        R['Y'] = clamp(prod(c1(p) for p in CELL_OUTPUT_PORTS[base][0]))

    elif base in ('OR2', 'OR3', 'OR4'):
        R['Y'] = clamp(1.0 - prod(c0(p) for p in CELL_OUTPUT_PORTS[base][0]))

    elif base in ('NAND2', 'NAND3', 'NAND4'):
        R['Y'] = clamp(1.0 - prod(c1(p) for p in CELL_OUTPUT_PORTS[base][0]))

    elif base in ('NOR2', 'NOR3', 'NOR4'):
        R['Y'] = clamp(prod(c0(p) for p in CELL_OUTPUT_PORTS[base][0]))

    elif base == 'INV':
        R['Y'] = clamp(c0('A'))

    elif base == 'BUF':
        R['Y'] = clamp(c1('A'))

    elif base == 'AOI21':
        # Y = ~( (A1&A2) | A3 )
        and_part = c1('A1') * c1('A2')
        R['Y'] = clamp((1.0 - and_part) * c0('A3'))

    elif base == 'AOI22':
        # Y = ~( (A1&A2) | (A3&A4) )
        and1 = c1('A1') * c1('A2')
        and2 = c1('A3') * c1('A4')
        R['Y'] = clamp((1.0 - and1) * (1.0 - and2))

    elif base == 'OAI21':
        # Y = ~( (A1|A2) & A3 )
        or_part = 1.0 - c0('A1') * c0('A2')
        R['Y'] = clamp(1.0 - or_part * c1('A3'))

    elif base == 'OAI22':
        # Y = ~( (A1|A2) & (A3|A4) )
        or1 = 1.0 - c0('A1') * c0('A2')
        or2 = 1.0 - c0('A3') * c0('A4')
        R['Y'] = clamp(1.0 - or1 * or2)

    elif base == 'OAI221':
        # Y = ~( (A1|A2) & (A3|A4) & A5 )
        or1 = 1.0 - c0('A1') * c0('A2')
        or2 = 1.0 - c0('A3') * c0('A4')
        R['Y'] = clamp(1.0 - or1 * or2 * c1('A5'))

    elif base == 'AO21':
        # Y = (A1&A2) | A3
        R['Y'] = clamp(1.0 - (1.0 - c1('A1') * c1('A2')) * c0('A3'))

    elif base == 'AO22':
        # Y = (A1&A2) | (A3&A4)
        and1 = c1('A1') * c1('A2')
        and2 = c1('A3') * c1('A4')
        R['Y'] = clamp(1.0 - (1.0 - and1) * (1.0 - and2))

    elif base == 'AO221':
        # Y = (A1&A2) | (A3&A4) | A5
        and1 = c1('A1') * c1('A2')
        and2 = c1('A3') * c1('A4')
        R['Y'] = clamp(1.0 - (1.0 - and1) * (1.0 - and2) * c0('A5'))

    elif base == 'AO222':
        # Y = (A1&A2) | (A3&A4) | (A5&A6)
        and1 = c1('A1') * c1('A2')
        and2 = c1('A3') * c1('A4')
        and3 = c1('A5') * c1('A6')
        R['Y'] = clamp(1.0 - (1.0 - and1) * (1.0 - and2) * (1.0 - and3))

    elif base == 'OA21':
        # Y = (A1|A2) & A3
        R['Y'] = clamp((1.0 - c0('A1') * c0('A2')) * c1('A3'))

    elif base == 'OA22':
        # Y = (A1|A2) & (A3|A4)
        or1 = 1.0 - c0('A1') * c0('A2')
        or2 = 1.0 - c0('A3') * c0('A4')
        R['Y'] = clamp(or1 * or2)

    elif base == 'OA221':
        # Y = (A1|A2) & (A3|A4) & ~... actually OA221: Y=((A1|A2)&(A3|A4))|A5
        or1 = 1.0 - c0('A1') * c0('A2')
        or2 = 1.0 - c0('A3') * c0('A4')
        R['Y'] = clamp(1.0 - (1.0 - or1 * or2) * c0('A5'))

    elif base == 'OA222':
        # Y = (A1|A2) & (A3|A4) & (A5|A6)
        or1 = 1.0 - c0('A1') * c0('A2')
        or2 = 1.0 - c0('A3') * c0('A4')
        or3 = 1.0 - c0('A5') * c0('A6')
        R['Y'] = clamp(or1 * or2 * or3)

    elif base == 'MUX21':
        # Y = S0 ? A2 : A1
        R['Y'] = clamp(c1('S0') * c1('A2') + c0('S0') * c1('A1'))

    elif base == 'MUX41':
        # Approximate: equal probability of each data input being selected
        R['Y'] = clamp(0.25 * (c1('A1') + c1('A2') + c1('A3') + c1('A4')))

    elif base == 'HADD':
        # SO = A0 XOR B0,  CO = A0 AND B0
        R['SO'] = clamp(c1('A0') * c0('B0') + c0('A0') * c1('B0'))
        R['CO'] = clamp(c1('A0') * c1('B0'))

    elif base == 'FADD':
        # S = A XOR B XOR CI
        xab     = c1('A') * c0('B') + c0('A') * c1('B')
        R['S']  = clamp(xab * c0('CI') + (1.0 - xab) * c1('CI'))
        R['CO'] = clamp(c1('A') * c1('B') +
                        c1('A') * c1('CI') +
                        c1('B') * c1('CI') -
                        2.0 * c1('A') * c1('B') * c1('CI'))

    elif base in ('SDFFX1', 'SDFFX2', 'DFFX1'):
        # Steady-state: Q tracks D
        q = c1('D')
        R['Q']  = clamp(q)
        R['QN'] = clamp(1.0 - q)

    elif base in ('NBUFF', 'IBUFF'):
        # Non-inverting buffer variants
        R['Y'] = clamp(c1('A'))

    elif base == 'AOI221':
        # Y = ~((A1&A2)|(A3&A4)|A5)
        and1 = c1('A1') * c1('A2')
        and2 = c1('A3') * c1('A4')
        R['Y'] = clamp((1.0 - and1) * (1.0 - and2) * c0('A5'))

    elif base == 'AOI222':
        # Y = ~((A1&A2)|(A3&A4)|(A5&A6))
        and1 = c1('A1') * c1('A2')
        and2 = c1('A3') * c1('A4')
        and3 = c1('A5') * c1('A6')
        R['Y'] = clamp((1.0 - and1) * (1.0 - and2) * (1.0 - and3))

    elif base == 'OAI222':
        # Y = ~((A1|A2)&(A3|A4)&(A5|A6))
        or1 = 1.0 - c0('A1') * c0('A2')
        or2 = 1.0 - c0('A3') * c0('A4')
        or3 = 1.0 - c0('A5') * c0('A6')
        R['Y'] = clamp(1.0 - or1 * or2 * or3)

    else:
        unknown_cells.add(base)
        for op in CELL_OUTPUT_PORTS.get(base, ([], ['Y']))[1]:
            R[op] = 0.5

    return R


# ---------------------------------------------------------------------------
# COP BACKWARD PASS — compute CO for each input given output CO values
# ---------------------------------------------------------------------------

def compute_co_inputs(base, input_cc1, co_outputs):
    """
    Return dict {input_port: co_value} for a gate.
    input_cc1  : dict {input_port: cc1_value}    (from forward pass)
    co_outputs : dict {output_port: co_value}     (already computed)
    """
    def c1(p): return input_cc1.get(p, 0.5)
    def c0(p): return 1.0 - input_cc1.get(p, 0.5)
    def co(op): return co_outputs.get(op, 0.0)
    R = {}

    if base in ('AND2', 'AND3', 'AND4'):
        ins  = CELL_OUTPUT_PORTS[base][0]
        co_y = co('Y')
        for p in ins:
            R[p] = clamp(co_y * prod(c1(q) for q in ins if q != p))

    elif base in ('OR2', 'OR3', 'OR4'):
        ins  = CELL_OUTPUT_PORTS[base][0]
        co_y = co('Y')
        for p in ins:
            R[p] = clamp(co_y * prod(c0(q) for q in ins if q != p))

    elif base in ('NAND2', 'NAND3', 'NAND4'):
        ins  = CELL_OUTPUT_PORTS[base][0]
        co_y = co('Y')
        for p in ins:
            R[p] = clamp(co_y * prod(c1(q) for q in ins if q != p))

    elif base in ('NOR2', 'NOR3', 'NOR4'):
        ins  = CELL_OUTPUT_PORTS[base][0]
        co_y = co('Y')
        for p in ins:
            R[p] = clamp(co_y * prod(c0(q) for q in ins if q != p))

    elif base == 'INV':
        R['A'] = clamp(co('Y'))

    elif base == 'BUF':
        R['A'] = clamp(co('Y'))

    elif base == 'AOI21':
        # Y = ~((A1&A2)|A3)
        co_y = co('Y')
        R['A1'] = clamp(co_y * c1('A2') * c0('A3'))
        R['A2'] = clamp(co_y * c1('A1') * c0('A3'))
        R['A3'] = clamp(co_y * (1.0 - c1('A1') * c1('A2')))

    elif base == 'AOI22':
        # Y = ~((A1&A2)|(A3&A4))
        and1 = c1('A1') * c1('A2')
        and2 = c1('A3') * c1('A4')
        co_y = co('Y')
        R['A1'] = clamp(co_y * c1('A2') * (1.0 - and2))
        R['A2'] = clamp(co_y * c1('A1') * (1.0 - and2))
        R['A3'] = clamp(co_y * c1('A4') * (1.0 - and1))
        R['A4'] = clamp(co_y * c1('A3') * (1.0 - and1))

    elif base == 'OAI21':
        # Y = ~((A1|A2)&A3)
        or12 = 1.0 - c0('A1') * c0('A2')
        co_y = co('Y')
        R['A1'] = clamp(co_y * c0('A2') * c1('A3'))
        R['A2'] = clamp(co_y * c0('A1') * c1('A3'))
        R['A3'] = clamp(co_y * or12)

    elif base == 'OAI22':
        # Y = ~((A1|A2)&(A3|A4))
        or1  = 1.0 - c0('A1') * c0('A2')
        or2  = 1.0 - c0('A3') * c0('A4')
        co_y = co('Y')
        R['A1'] = clamp(co_y * c0('A2') * or2)
        R['A2'] = clamp(co_y * c0('A1') * or2)
        R['A3'] = clamp(co_y * c0('A4') * or1)
        R['A4'] = clamp(co_y * c0('A3') * or1)

    elif base == 'OAI221':
        # Y = ~((A1|A2)&(A3|A4)&A5)
        or1  = 1.0 - c0('A1') * c0('A2')
        or2  = 1.0 - c0('A3') * c0('A4')
        co_y = co('Y')
        R['A1'] = clamp(co_y * c0('A2') * or2  * c1('A5'))
        R['A2'] = clamp(co_y * c0('A1') * or2  * c1('A5'))
        R['A3'] = clamp(co_y * c0('A4') * or1  * c1('A5'))
        R['A4'] = clamp(co_y * c0('A3') * or1  * c1('A5'))
        R['A5'] = clamp(co_y * or1 * or2)

    elif base == 'AO21':
        # Y = (A1&A2)|A3
        co_y = co('Y')
        R['A1'] = clamp(co_y * c1('A2') * c0('A3'))
        R['A2'] = clamp(co_y * c1('A1') * c0('A3'))
        R['A3'] = clamp(co_y * (1.0 - c1('A1') * c1('A2')))

    elif base == 'AO22':
        # Y = (A1&A2)|(A3&A4)
        and1 = c1('A1') * c1('A2')
        and2 = c1('A3') * c1('A4')
        co_y = co('Y')
        R['A1'] = clamp(co_y * c1('A2') * (1.0 - and2))
        R['A2'] = clamp(co_y * c1('A1') * (1.0 - and2))
        R['A3'] = clamp(co_y * c1('A4') * (1.0 - and1))
        R['A4'] = clamp(co_y * c1('A3') * (1.0 - and1))

    elif base == 'AO221':
        # Y = (A1&A2)|(A3&A4)|A5
        and1 = c1('A1') * c1('A2')
        and2 = c1('A3') * c1('A4')
        co_y = co('Y')
        R['A1'] = clamp(co_y * c1('A2') * (1.0 - and2) * c0('A5'))
        R['A2'] = clamp(co_y * c1('A1') * (1.0 - and2) * c0('A5'))
        R['A3'] = clamp(co_y * c1('A4') * (1.0 - and1) * c0('A5'))
        R['A4'] = clamp(co_y * c1('A3') * (1.0 - and1) * c0('A5'))
        R['A5'] = clamp(co_y * (1.0 - (1.0 - and1) * (1.0 - and2)))

    elif base == 'AO222':
        # Y = (A1&A2)|(A3&A4)|(A5&A6)
        and1 = c1('A1') * c1('A2')
        and2 = c1('A3') * c1('A4')
        and3 = c1('A5') * c1('A6')
        co_y = co('Y')
        R['A1'] = clamp(co_y * c1('A2') * (1.0 - and2) * (1.0 - and3))
        R['A2'] = clamp(co_y * c1('A1') * (1.0 - and2) * (1.0 - and3))
        R['A3'] = clamp(co_y * c1('A4') * (1.0 - and1) * (1.0 - and3))
        R['A4'] = clamp(co_y * c1('A3') * (1.0 - and1) * (1.0 - and3))
        R['A5'] = clamp(co_y * c1('A6') * (1.0 - and1) * (1.0 - and2))
        R['A6'] = clamp(co_y * c1('A5') * (1.0 - and1) * (1.0 - and2))

    elif base == 'OA21':
        # Y = (A1|A2)&A3
        co_y = co('Y')
        R['A1'] = clamp(co_y * c0('A2') * c1('A3'))
        R['A2'] = clamp(co_y * c0('A1') * c1('A3'))
        R['A3'] = clamp(co_y * (1.0 - c0('A1') * c0('A2')))

    elif base == 'OA22':
        # Y = (A1|A2)&(A3|A4)
        or1  = 1.0 - c0('A1') * c0('A2')
        or2  = 1.0 - c0('A3') * c0('A4')
        co_y = co('Y')
        R['A1'] = clamp(co_y * c0('A2') * or2)
        R['A2'] = clamp(co_y * c0('A1') * or2)
        R['A3'] = clamp(co_y * c0('A4') * or1)
        R['A4'] = clamp(co_y * c0('A3') * or1)

    elif base == 'OA221':
        # Y = ((A1|A2)&(A3|A4))|A5
        or1  = 1.0 - c0('A1') * c0('A2')
        or2  = 1.0 - c0('A3') * c0('A4')
        co_y = co('Y')
        R['A1'] = clamp(co_y * c0('A2') * or2 * c0('A5'))
        R['A2'] = clamp(co_y * c0('A1') * or2 * c0('A5'))
        R['A3'] = clamp(co_y * c0('A4') * or1 * c0('A5'))
        R['A4'] = clamp(co_y * c0('A3') * or1 * c0('A5'))
        R['A5'] = clamp(co_y * (1.0 - (1.0 - or1 * or2)))

    elif base == 'OA222':
        # Y = (A1|A2)&(A3|A4)&(A5|A6)
        or1  = 1.0 - c0('A1') * c0('A2')
        or2  = 1.0 - c0('A3') * c0('A4')
        or3  = 1.0 - c0('A5') * c0('A6')
        co_y = co('Y')
        R['A1'] = clamp(co_y * c0('A2') * or2 * or3)
        R['A2'] = clamp(co_y * c0('A1') * or2 * or3)
        R['A3'] = clamp(co_y * c0('A4') * or1 * or3)
        R['A4'] = clamp(co_y * c0('A3') * or1 * or3)
        R['A5'] = clamp(co_y * c0('A6') * or1 * or2)
        R['A6'] = clamp(co_y * c0('A5') * or1 * or2)

    elif base == 'MUX21':
        # Y = S0?A2:A1
        co_y = co('Y')
        R['A1'] = clamp(co_y * c0('S0'))
        R['A2'] = clamp(co_y * c1('S0'))
        R['S0'] = clamp(co_y * abs(c1('A1') - c1('A2')))

    elif base == 'MUX41':
        co_y = co('Y')
        for p in ('A1', 'A2', 'A3', 'A4'):
            R[p] = clamp(co_y * 0.25)
        R['S0'] = clamp(co_y * 0.5)
        R['S1'] = clamp(co_y * 0.5)

    elif base == 'HADD':
        co_so = co('SO')
        co_co = co('CO')
        R['A0'] = clamp(co_so + co_co * c1('B0'))
        R['B0'] = clamp(co_so + co_co * c1('A0'))

    elif base == 'FADD':
        co_s  = co('S')
        co_co = co('CO')
        R['A']  = clamp(co_s + co_co * (c1('B')  + c1('CI') - c1('B')  * c1('CI')))
        R['B']  = clamp(co_s + co_co * (c1('A')  + c1('CI') - c1('A')  * c1('CI')))
        R['CI'] = clamp(co_s + co_co * (c1('A')  + c1('B')  - c1('A')  * c1('B')))

    elif base in ('SDFFX1', 'SDFFX2', 'DFFX1'):
        R['D'] = clamp(co('Q') + co('QN'))

    elif base in ('NBUFF', 'IBUFF'):
        R['A'] = clamp(co('Y'))

    elif base == 'AOI221':
        # Y = ~((A1&A2)|(A3&A4)|A5)
        and1 = c1('A1') * c1('A2')
        and2 = c1('A3') * c1('A4')
        co_y = co('Y')
        R['A1'] = clamp(co_y * c1('A2') * (1.0 - and2) * c0('A5'))
        R['A2'] = clamp(co_y * c1('A1') * (1.0 - and2) * c0('A5'))
        R['A3'] = clamp(co_y * c1('A4') * (1.0 - and1) * c0('A5'))
        R['A4'] = clamp(co_y * c1('A3') * (1.0 - and1) * c0('A5'))
        R['A5'] = clamp(co_y * (1.0 - (1.0 - and1) * (1.0 - and2)))

    elif base == 'AOI222':
        # Y = ~((A1&A2)|(A3&A4)|(A5&A6))
        and1 = c1('A1') * c1('A2')
        and2 = c1('A3') * c1('A4')
        and3 = c1('A5') * c1('A6')
        co_y = co('Y')
        R['A1'] = clamp(co_y * c1('A2') * (1.0 - and2) * (1.0 - and3))
        R['A2'] = clamp(co_y * c1('A1') * (1.0 - and2) * (1.0 - and3))
        R['A3'] = clamp(co_y * c1('A4') * (1.0 - and1) * (1.0 - and3))
        R['A4'] = clamp(co_y * c1('A3') * (1.0 - and1) * (1.0 - and3))
        R['A5'] = clamp(co_y * c1('A6') * (1.0 - and1) * (1.0 - and2))
        R['A6'] = clamp(co_y * c1('A5') * (1.0 - and1) * (1.0 - and2))

    elif base == 'OAI222':
        # Y = ~((A1|A2)&(A3|A4)&(A5|A6))
        or1  = 1.0 - c0('A1') * c0('A2')
        or2  = 1.0 - c0('A3') * c0('A4')
        or3  = 1.0 - c0('A5') * c0('A6')
        co_y = co('Y')
        R['A1'] = clamp(co_y * c0('A2') * or2 * or3)
        R['A2'] = clamp(co_y * c0('A1') * or2 * or3)
        R['A3'] = clamp(co_y * c0('A4') * or1 * or3)
        R['A4'] = clamp(co_y * c0('A3') * or1 * or3)
        R['A5'] = clamp(co_y * c0('A6') * or1 * or2)
        R['A6'] = clamp(co_y * c0('A5') * or1 * or2)

    return R


# ---------------------------------------------------------------------------
# BUILD GRAPH + TOPOLOGICAL SORT  (Kahn's algorithm)
# ---------------------------------------------------------------------------

def build_and_sort(ports_in, instances, assigns):
    """
    Returns:
        topo        : list of instance indices in PI->PO topological order
        net_driver  : dict  net -> instance_idx  (None = driven by PI)
        inst_inputs : list of lists  inst_idx -> [resolved_net, ...]
        assign_map  : dict  lhs_net -> rhs_net
    """
    assign_map = {lhs: rhs for lhs, rhs in assigns}

    def resolve(net):
        """Follow assign aliases to ultimate driver net."""
        visited = set()
        while net in assign_map and net not in visited:
            visited.add(net)
            net = assign_map[net]
        return net

    n          = len(instances)
    net_driver = {pi: None for pi in ports_in}   # None = driven by PI
    inst_inputs = [[] for _ in range(n)]

    for idx, inst in enumerate(instances):
        base = get_cell_base(inst['cell'])
        info = CELL_OUTPUT_PORTS.get(base)
        if info is None:
            unknown_cells.add(base)
            continue
        in_ports, out_ports = info
        conn = inst['conn']

        # Register output nets as driven by this instance
        for op in out_ports:
            net = conn.get(op)
            if net:
                net_driver[net] = idx

        # Record (resolved) input nets
        for ip in in_ports:
            net = conn.get(ip)
            if net:
                inst_inputs[idx].append(resolve(net))

    # Build gate-to-gate dependency graph
    gate_fanout = defaultdict(list)
    in_deg      = [0] * n

    # Flip-flops are topo roots: their Q/QN are seeded from the scan chain,
    # not from combinational logic within the same cycle. Treating them as
    # dependencies would create false cycles through sequential feedback loops.
    ff_set = set()
    for idx, inst in enumerate(instances):
        if get_cell_base(inst["cell"]) in ("SDFFX1", "SDFFX2", "DFFX1"):
            ff_set.add(idx)

    for idx in range(n):
        if idx in ff_set:
            continue   # FFs start with in_deg=0 (topo roots)
        seen_deps = set()
        for rnet in inst_inputs[idx]:
            drv = net_driver.get(rnet)
            if drv is not None and drv not in seen_deps:
                seen_deps.add(drv)
                in_deg[idx] += 1
                gate_fanout[drv].append(idx)

    # Kahn's BFS topological sort
    queue = deque(i for i in range(n) if in_deg[i] == 0)
    topo  = []
    while queue:
        i = queue.popleft()
        topo.append(i)
        for j in gate_fanout[i]:
            in_deg[j] -= 1
            if in_deg[j] == 0:
                queue.append(j)

    if len(topo) != n:
        n_skipped = n - len(topo)
        print('  WARNING: {} instances excluded from topo sort '
              '(unresolved inputs or unknown cells).'.format(n_skipped))

    return topo, net_driver, inst_inputs, assign_map


# ---------------------------------------------------------------------------
# COP MAIN
# ---------------------------------------------------------------------------

def run_cop(ports_in, ports_out, instances, assigns):
    """
    Run the full COP forward + backward pass.
    Returns:
        cc1_vals : dict  net -> 1-controllability  [0, 1]
        co_vals  : dict  net -> observability      [0, 1]
    """
    topo, net_driver, inst_inputs, assign_map = \
        build_and_sort(ports_in, instances, assigns)

    def resolve(net):
        visited = set()
        while net in assign_map and net not in visited:
            visited.add(net)
            net = assign_map[net]
        return net

    # ----------------------------------------------------------------
    # FORWARD PASS: CC1
    # ----------------------------------------------------------------
    cc1 = {pi: 0.5 for pi in ports_in}

    for idx in topo:
        inst = instances[idx]
        base = get_cell_base(inst['cell'])
        info = CELL_OUTPUT_PORTS.get(base)
        if info is None:
            continue
        in_ports, out_ports = info
        conn = inst['conn']

        input_cc1 = {}
        for ip in in_ports:
            net = conn.get(ip)
            if net:
                input_cc1[ip] = cc1.get(resolve(net), 0.5)

        for op, val in compute_cc1(base, input_cc1).items():
            net = conn.get(op)
            if net:
                cc1[net] = val

    # Propagate through assigns
    for lhs, rhs in assigns:
        if rhs in cc1 and lhs not in cc1:
            cc1[lhs] = cc1[rhs]

    # ----------------------------------------------------------------
    # BACKWARD PASS: CO
    # ----------------------------------------------------------------
    co = {}

    # Primary outputs are directly observable
    for po in ports_out:
        co[po] = 1.0

    # Flip-flop outputs are observable through the scan chain
    for inst in instances:
        base = get_cell_base(inst['cell'])
        if base in ('SDFFX1', 'SDFFX2', 'DFFX1'):
            for op in ('Q', 'QN'):
                net = inst['conn'].get(op)
                if net:
                    co[net] = max(co.get(net, 0.0), 1.0)

    # Propagate CO backward through assign aliases before gate sweep
    # so internal nets driven by POs start with CO=1
    for lhs, rhs in assigns:
        if lhs in co:
            co[rhs] = max(co.get(rhs, 0.0), co[lhs])

    # Iterative backward sweep until convergence.
    # A single reversed-topo pass is insufficient when CO values flow
    # through sequential feedback paths (FF Q -> combinational -> FF D)
    # because the FF D input observability depends on CO already set at
    # the FF Q output, which may not yet be visible when the feeding gate
    # is first processed. Two to three iterations always suffice in practice.
    MAX_ITER = 10
    for _ in range(MAX_ITER):
        changed = False
        for idx in reversed(topo):
            inst = instances[idx]
            base = get_cell_base(inst['cell'])
            info = CELL_OUTPUT_PORTS.get(base)
            if info is None:
                continue
            in_ports, out_ports = info
            conn = inst['conn']

            input_cc1 = {}
            for ip in in_ports:
                net = conn.get(ip)
                if net:
                    input_cc1[ip] = cc1.get(resolve(net), 0.5)

            co_outputs = {}
            for op in out_ports:
                net = conn.get(op)
                if net:
                    co_outputs[op] = co.get(net, 0.0)

            for ip, val in compute_co_inputs(base, input_cc1, co_outputs).items():
                net = conn.get(ip)
                if net:
                    rnet = resolve(net)
                    old = co.get(rnet, 0.0)
                    new = clamp(max(old, val))
                    if new > old + 1e-9:
                        co[rnet] = new
                        changed = True

        # Propagate backwards through assigns each iteration
        for lhs, rhs in reversed(assigns):
            if lhs in co:
                old = co.get(rhs, 0.0)
                new = max(old, co[lhs])
                if new > old + 1e-9:
                    co[rhs] = new
                    changed = True

        if not changed:
            break

    return cc1, co


# ---------------------------------------------------------------------------
# CANDIDATE IDENTIFICATION  (SPAR Section IV)
# ---------------------------------------------------------------------------

def identify_candidates(cps, ops, cc1_vals, co_vals, pin_to_net, verbose=True):
    """
    Classify test points as EO-CP or EC-OP using CCTh and COTh.

    EO-CP : CP whose output net has CO > COTh  (easily observable CP)
    EC-OP : OP whose output net has CCTh < CC1 < 1-CCTh  (easily controllable OP)

    CCTh = max( avg CC1 of OR-CPs,  avg(1 - CC1) of AND-CPs )
    COTh = avg CO of OPs
    """
    lookup = make_lookup(pin_to_net, cc1_vals)

    def get_cc1(ref):
        net = lookup(ref)
        return cc1_vals[net] if (net and net in cc1_vals) else None

    def get_co(ref):
        net = lookup(ref)
        return co_vals[net] if (net and net in co_vals) else None

    or_cps  = [r for r, t in cps if t == 'control_1']
    and_cps = [r for r, t in cps if t == 'control_0']

    or_cc1s    = [v for r in or_cps  for v in [get_cc1(r)] if v is not None]
    and_cc1s   = [v for r in and_cps for v in [get_cc1(r)] if v is not None]
    op_co_list = [v for r in ops     for v in [get_co(r)]  if v is not None]

    avg_cc_or    = sum(or_cc1s)              / len(or_cc1s)   if or_cc1s    else 0.0
    avg_1mcc_and = sum(1.0 - v for v in and_cc1s) / len(and_cc1s) if and_cc1s else 0.0
    CC_Th = max(avg_cc_or, avg_1mcc_and)
    CO_Th = sum(op_co_list) / len(op_co_list) if op_co_list else 0.0

    if verbose:
        print('\n--- Thresholds (SPAR Section IV) ---')
        print('  CCTh = max(avg CC1 of OR-CPs={:.4f}, '
              'avg(1-CC1) of AND-CPs={:.4f}) = {:.4f}'.format(
                  avg_cc_or, avg_1mcc_and, CC_Th))
        print('  COTh = avg CO of OPs = {:.4f}'.format(CO_Th))

    eo_cps = []
    for ref, tp_type in cps:
        obs = get_co(ref)
        if obs is not None and obs > CO_Th:
            eo_cps.append((ref, tp_type, obs))

    ec_ops = []
    for ref in ops:
        ctrl = get_cc1(ref)
        if ctrl is not None and CC_Th < ctrl < (1.0 - CC_Th):
            ec_ops.append((ref, ctrl))

    return eo_cps, ec_ops, CC_Th, CO_Th


# ---------------------------------------------------------------------------
# MAIN
# ---------------------------------------------------------------------------

def main():
    import argparse
    parser = argparse.ArgumentParser(
        description='COP testability analysis for SPAR replication')
    parser.add_argument('netlist',           help='Gate-level Verilog (.v)')
    parser.add_argument('--tp_file',         help='TetraMAX tp_analysis.txt',
                        default=None)
    args = parser.parse_args()

    # ---- Parse ----
    print('Parsing:', args.netlist)
    ports_in, ports_out, instances, assigns, pin_to_net = \
        parse_verilog(args.netlist)
    print('  PIs={}  POs={}  instances={}  assigns={}'.format(
        len(ports_in), len(ports_out), len(instances), len(assigns)))

    # ---- COP ----
    print('Running COP...')
    cc1_vals, co_vals = run_cop(ports_in, ports_out, instances, assigns)
    print('  CC1 nets={}  CO nets={}'.format(len(cc1_vals), len(co_vals)))
    if unknown_cells:
        print('  WARNING - unrecognised cells (treated as 0.5):', sorted(unknown_cells))

    # ---- Sanity checks ----
    print('\n--- PI sanity (CC1 should be 0.5) ---')
    for pi in sorted(ports_in)[:5]:
        print('  PI {:15s}  CC1={:.4f}  CO={:.4f}'.format(
            pi, cc1_vals.get(pi, -1), co_vals.get(pi, -1)))

    print('\n--- PO sanity (CO should be 1.0) ---')
    for po in sorted(ports_out)[:5]:
        print('  PO {:15s}  CC1={:.4f}  CO={:.4f}'.format(
            po, cc1_vals.get(po, -1), co_vals.get(po, -1)))

    if not args.tp_file:
        return

    # ---- Test points ----
    print('\nParsing TP file:', args.tp_file)
    cps, ops = parse_tp_file(args.tp_file)
    print('  CPs={}  OPs={}'.format(len(cps), len(ops)))

    lookup = make_lookup(pin_to_net, cc1_vals)

    print('\n--- CC1 and CO for CPs ---')
    for ref, tp_type in cps:
        net  = lookup(ref)
        ctrl = cc1_vals.get(net, -1) if net else -1
        obs  = co_vals.get(net,  -1) if net else -1
        label = 'OR-CP' if tp_type == 'control_1' else 'AND-CP'
        print('  {:7s}  {:35s}  (net={:12s})  CC1={:.4f}  CO={:.6f}'.format(
            label, ref, str(net), ctrl, obs))

    print('\n--- CC1 and CO for OPs ---')
    for ref in ops:
        net  = lookup(ref)
        ctrl = cc1_vals.get(net, -1) if net else -1
        obs  = co_vals.get(net,  -1) if net else -1
        print('  OP  {:35s}  (net={:12s})  CC1={:.4f}  CO={:.6f}'.format(
            ref, str(net), ctrl, obs))

    # ---- Candidate classification ----
    eo_cps, ec_ops, CC_Th, CO_Th = identify_candidates(
        cps, ops, cc1_vals, co_vals, pin_to_net)

    print('\n--- EO-CP candidates  (CO > {:.4f}) ---'.format(CO_Th))
    if eo_cps:
        for ref, tp_type, obs in eo_cps:
            label = 'OR-CP' if tp_type == 'control_1' else 'AND-CP'
            print('  {:7s}  {:35s}  CO={:.6f}'.format(label, ref, obs))
    else:
        print('  (none)')

    print('\n--- EC-OP candidates  ({:.4f} < CC1 < {:.4f}) ---'.format(
        CC_Th, 1.0 - CC_Th))
    if ec_ops:
        for ref, ctrl in ec_ops:
            print('  OP  {:35s}  CC1={:.4f}'.format(ref, ctrl))
    else:
        print('  (none)')

    print('\nSummary: EO-CPs={}  EC-OPs={}'.format(len(eo_cps), len(ec_ops)))


if __name__ == '__main__':
    main()
