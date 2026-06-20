"""
COP (Controllability/Observability Probability) implementation
for SAED32nm gate-level Verilog netlists synthesized by Synopsys DC.

Based on: Brglez et al., "Applications of testability analysis:
From ATPG to critical delay path tracing," ITC 1984.

Usage:
    python cop.py s9234.v [--tp_file s9234_tp_analysis.txt]
"""

import re
import sys
from collections import defaultdict, deque

# ---------------------------------------------------------------------------
# 1.  CELL TYPE -> COP FORMULA
#     Each entry maps a cell base-name to:
#       inputs  : list of logical input port names
#       output  : logical output port name
#       cc1(ins): 1-controllability formula  (product / sum rules)
#       co(out, ins, co_out): observability formula
#
#     COP rules (Brglez 1984):
#       AND  : cc1 = prod(cc1_i)       co_i = co_out * prod_{j!=i}(cc1_j)
#       OR   : cc1 = 1-prod(cc0_i)     co_i = co_out * prod_{j!=i}(cc0_j)
#       NAND : cc1 = 1-prod(cc1_i)     co_i = co_out * prod_{j!=i}(cc1_j)
#       NOR  : cc1 = prod(cc0_i)       co_i = co_out * prod_{j!=i}(cc0_j)
#       INV  : cc1 = cc0_in = 1-cc1_in co_i = co_out
#       BUF  : cc1 = cc1_in            co_i = co_out
#       XOR  (2-in): cc1=cc1_a*cc0_b + cc0_a*cc1_b
#       XNOR (2-in): cc1=cc1_a*cc1_b + cc0_a*cc0_b
#       MUX  (A1,A2,S0->Y): handled explicitly
#       HADD (A0,B0->SO,CO): handled explicitly
#       FADD (A,B,CI->S,CO): handled explicitly
# ---------------------------------------------------------------------------

def prod(vals):
    r = 1.0
    for v in vals:
        r *= v
    return r

def clamp(v):
    return max(0.0, min(1.0, v))


# ---------------------------------------------------------------------------
# 2.  NETLIST PARSER
# ---------------------------------------------------------------------------

def parse_verilog(filename):
    """
    Parse a flat gate-level Verilog netlist produced by Synopsys DC.
    Returns:
        ports_in  : set of input port net names
        ports_out : set of output port net names
        instances : list of dicts with keys:
                      cell, inst, connections {port: net}
        assigns   : list of (lhs_net, rhs_net)
    """
    with open(filename, 'r') as f:
        text = f.read()

    # Remove line-continuation by joining continuation lines
    # (DC wraps long lines; we join them for easier parsing)
    text = re.sub(r'\\\n', ' ', text)

    # Remove block comments
    text = re.sub(r'/\*.*?\*/', ' ', text, flags=re.DOTALL)
    # Remove line comments
    text = re.sub(r'//.*', '', text)

    # ---- port declarations ----
    input_re  = re.compile(r'\binput\b([^;]+);')
    output_re = re.compile(r'\boutput\b([^;]+);')

    ports_in  = set()
    ports_out = set()

    for m in input_re.finditer(text):
        for net in re.split(r'[,\s]+', m.group(1)):
            net = net.strip()
            if net:
                ports_in.add(net)

    for m in output_re.finditer(text):
        for net in re.split(r'[,\s]+', m.group(1)):
            net = net.strip()
            if net:
                ports_out.add(net)

    # ---- assign statements ----
    assign_re = re.compile(r'\bassign\s+(\S+)\s*=\s*(\S+)\s*;')
    assigns = []
    for m in assign_re.finditer(text):
        assigns.append((m.group(1), m.group(2)))

    # ---- gate instances ----
    # Pattern: CELLTYPE INSTNAME ( .PORT(NET), ... );
    # Cell names in SAED32 end with _LVT
    inst_re = re.compile(
        r'(\w+_LVT)\s+(\\?[\w/\[\]]+)\s*\(([^;]+)\);',
        re.DOTALL
    )
    port_re = re.compile(r'\.(\w+)\s*\(([^)]*)\)')

    instances = []
    for m in inst_re.finditer(text):
        cell = m.group(1)
        inst = m.group(2)
        conn_str = m.group(3)
        connections = {}
        for pm in port_re.finditer(conn_str):
            port = pm.group(1)
            net  = pm.group(2).strip()
            if net:
                connections[port] = net
        instances.append({'cell': cell, 'inst': inst, 'conn': connections})

    return ports_in, ports_out, instances, assigns


# ---------------------------------------------------------------------------
# 3.  BUILD GRAPH  (net -> driving cell,  net -> fanout cells)
# ---------------------------------------------------------------------------

def get_cell_base(cell_name):
    """Strip drive strength and _LVT suffix: NAND2X0_LVT -> NAND2"""
    # Remove _LVT
    name = re.sub(r'_LVT$', '', cell_name)
    # Remove trailing drive strength (X0, X1, X2, ...)
    name = re.sub(r'X\d+$', '', name)
    return name


# Map cell base name -> (input_ports, output_ports)
# Output ports listed here are the SIGNAL outputs (not clock/scan).
CELL_OUTPUT_PORTS = {
    # Simple gates
    'AND2':    (['A1','A2'],          ['Y']),
    'AND3':    (['A1','A2','A3'],     ['Y']),
    'AND4':    (['A1','A2','A3','A4'],['Y']),
    'OR2':     (['A1','A2'],          ['Y']),
    'OR3':     (['A1','A2','A3'],     ['Y']),
    'OR4':     (['A1','A2','A3','A4'],['Y']),
    'NAND2':   (['A1','A2'],          ['Y']),
    'NAND3':   (['A1','A2','A3'],     ['Y']),
    'NAND4':   (['A1','A2','A3','A4'],['Y']),
    'NOR2':    (['A1','A2'],          ['Y']),
    'NOR3':    (['A1','A2','A3'],     ['Y']),
    'NOR4':    (['A1','A2','A3','A4'],['Y']),
    'INV':     (['A'],                ['Y']),
    'BUF':     (['A'],                ['Y']),
    # AOI/OAI  (AND-OR-INV / OR-AND-INV)
    # AOI21: Y = ~((A1&A2)|A3)   inputs: A1,A2,A3
    'AOI21':   (['A1','A2','A3'],     ['Y']),
    'AOI22':   (['A1','A2','A3','A4'],['Y']),
    'OAI21':   (['A1','A2','A3'],     ['Y']),
    'OAI22':   (['A1','A2','A3','A4'],['Y']),
    'OAI221':  (['A1','A2','A3','A4','A5'], ['Y']),
    # AO/OA (non-inverting)
    'AO21':    (['A1','A2','A3'],     ['Y']),
    'AO22':    (['A1','A2','A3','A4'],['Y']),
    'AO221':   (['A1','A2','A3','A4','A5'],['Y']),
    'AO222':   (['A1','A2','A3','A4','A5','A6'],['Y']),
    'OA21':    (['A1','A2','A3'],     ['Y']),
    'OA22':    (['A1','A2','A3','A4'],['Y']),
    'OA221':   (['A1','A2','A3','A4','A5'],['Y']),
    'OA222':   (['A1','A2','A3','A4','A5','A6'],['Y']),
    # MUX
    'MUX21':   (['A1','A2','S0'],     ['Y']),
    'MUX41':   (['A1','A2','A3','A4','S0','S1'], ['Y']),
    # Adders  - multiple outputs
    'HADD':    (['A0','B0'],          ['SO','CO']),
    'FADD':    (['A','B','CI'],       ['S','CO']),
    # Flip-flops - Q is the combinational output for COP purposes
    'SDFFX2':  (['D'],                ['Q','QN']),   # scan FF
    'DFFX1':   (['D'],                ['Q','QN']),   # non-scan FF
}
unknown_cells = set()


def compute_cc1(base, input_cc1, conn):
    """
    Compute 1-controllability of each output of a gate.
    Returns dict {output_port: cc1_value}.
    input_cc1: dict {port: cc1}
    conn: dict {port: net}  (used only for MUX select logic)
    """
    def cc0(port):
        return 1.0 - input_cc1.get(port, 0.5)
    def cc1(port):
        return input_cc1.get(port, 0.5)

    result = {}

    if base in ('AND2','AND3','AND4'):
        ins = [p for p in CELL_OUTPUT_PORTS[base][0]]
        result['Y'] = clamp(prod(cc1(p) for p in ins))

    elif base in ('OR2','OR3','OR4'):
        ins = CELL_OUTPUT_PORTS[base][0]
        result['Y'] = clamp(1.0 - prod(cc0(p) for p in ins))

    elif base in ('NAND2','NAND3','NAND4'):
        ins = CELL_OUTPUT_PORTS[base][0]
        result['Y'] = clamp(1.0 - prod(cc1(p) for p in ins))

    elif base in ('NOR2','NOR3','NOR4'):
        ins = CELL_OUTPUT_PORTS[base][0]
        result['Y'] = clamp(prod(cc0(p) for p in ins))

    elif base == 'INV':
        result['Y'] = clamp(cc0('A'))

    elif base == 'BUF':
        result['Y'] = clamp(cc1('A'))

    # AOI21: Y = ~((A1&A2)|A3)  => cc1(Y) = cc0((A1&A2)|A3)
    elif base == 'AOI21':
        and_part = cc1('A1') * cc1('A2')
        or_cc1   = 1.0 - (1.0 - and_part) * cc0('A3')
        result['Y'] = clamp(1.0 - or_cc1)

    elif base == 'AOI22':
        and1 = cc1('A1') * cc1('A2')
        and2 = cc1('A3') * cc1('A4')
        or_cc1 = 1.0 - (1.0 - and1) * (1.0 - and2)
        result['Y'] = clamp(1.0 - or_cc1)

    # OAI21: Y = ~((A1|A2)&A3)
    elif base == 'OAI21':
        or_part  = 1.0 - cc0('A1') * cc0('A2')
        and_cc1  = or_part * cc1('A3')
        result['Y'] = clamp(1.0 - and_cc1)

    elif base == 'OAI22':
        or1 = 1.0 - cc0('A1') * cc0('A2')
        or2 = 1.0 - cc0('A3') * cc0('A4')
        result['Y'] = clamp(1.0 - or1 * or2)

    elif base == 'OAI221':
        or1 = 1.0 - cc0('A1') * cc0('A2')
        or2 = 1.0 - cc0('A3') * cc0('A4')
        result['Y'] = clamp(1.0 - or1 * or2 * cc1('A5'))

    # AO21: Y = (A1&A2)|A3
    elif base == 'AO21':
        and_part = cc1('A1') * cc1('A2')
        result['Y'] = clamp(1.0 - (1.0 - and_part) * cc0('A3'))

    elif base == 'AO22':
        and1 = cc1('A1') * cc1('A2')
        and2 = cc1('A3') * cc1('A4')
        result['Y'] = clamp(1.0 - (1.0 - and1) * (1.0 - and2))

    elif base == 'AO221':
        and1 = cc1('A1') * cc1('A2')
        and2 = cc1('A3') * cc1('A4')
        result['Y'] = clamp(1.0 - (1.0 - and1) * (1.0 - and2) * cc0('A5'))

    elif base == 'AO222':
        and1 = cc1('A1') * cc1('A2')
        and2 = cc1('A3') * cc1('A4')
        and3 = cc1('A5') * cc1('A6')
        result['Y'] = clamp(1.0 - (1.0-and1)*(1.0-and2)*(1.0-and3))

    # OA21: Y = (A1|A2)&A3
    elif base == 'OA21':
        or_part = 1.0 - cc0('A1') * cc0('A2')
        result['Y'] = clamp(or_part * cc1('A3'))

    elif base == 'OA22':
        or1 = 1.0 - cc0('A1') * cc0('A2')
        or2 = 1.0 - cc0('A3') * cc0('A4')
        result['Y'] = clamp(or1 * or2)

    elif base == 'OA221':
        or1 = 1.0 - cc0('A1') * cc0('A2')
        or2 = 1.0 - cc0('A3') * cc0('A4')
        result['Y'] = clamp(1.0 - (1.0 - or1*or2) * cc0('A5'))

    elif base == 'OA222':
        or1 = 1.0 - cc0('A1') * cc0('A2')
        or2 = 1.0 - cc0('A3') * cc0('A4')
        or3 = 1.0 - cc0('A5') * cc0('A6')
        result['Y'] = clamp(or1 * or2 * or3)

    # MUX21: Y = S0?A2:A1
    elif base == 'MUX21':
        result['Y'] = clamp(cc1('S0') * cc1('A2') + cc0('S0') * cc1('A1'))

    # MUX41: Y = sel(S1,S0) -> one of A1..A4
    elif base == 'MUX41':
        # approximate: average of all data inputs weighted by select prob
        result['Y'] = clamp(0.25*(cc1('A1')+cc1('A2')+cc1('A3')+cc1('A4')))

    # HADD: SO = A0 XOR B0
    elif base == 'HADD':
        result['SO'] = clamp(cc1('A0')*cc0('B0') + cc0('A0')*cc1('B0'))
        result['CO'] = clamp(cc1('A0') * cc1('B0'))

    # FADD: S = A XOR B XOR CI
    elif base == 'FADD':
        # Approximate XOR chain
        xor_ab = cc1('A')*cc0('B') + cc0('A')*cc1('B')
        result['S']  = clamp(xor_ab*cc0('CI') + (1.0-xor_ab)*cc1('CI'))
        result['CO'] = clamp(cc1('A')*cc1('B') +
                             cc1('A')*cc1('CI') +
                             cc1('B')*cc1('CI') -
                             2.0*cc1('A')*cc1('B')*cc1('CI'))

    # Flip-flops: treat Q as having cc1 = cc1(D) (steady-state approx)
    elif base in ('SDFFX2', 'DFFX1'):
        q_cc1 = cc1('D')
        result['Q']  = clamp(q_cc1)
        result['QN'] = clamp(1.0 - q_cc1)

    else:
        # Unknown cell: pass through 0.5
        for op in CELL_OUTPUT_PORTS.get(base, ([], ['Y']))[1]:
            result[op] = 0.5

    return result


def compute_co_inputs(base, input_cc1, co_outputs):
    """
    Compute observability of each INPUT of a gate given:
      input_cc1  : dict {input_port: cc1}
      co_outputs : dict {output_port: co}   (already computed)
    Returns dict {input_port: co_value}.
    """
    def cc0(port):
        return 1.0 - input_cc1.get(port, 0.5)
    def cc1(port):
        return input_cc1.get(port, 0.5)
    def co(oport):
        return co_outputs.get(oport, 0.0)

    result = {}

    if base in ('AND2','AND3','AND4'):
        ins = CELL_OUTPUT_PORTS[base][0]
        co_y = co('Y')
        for p in ins:
            others = [q for q in ins if q != p]
            result[p] = clamp(co_y * prod(cc1(q) for q in others))

    elif base in ('OR2','OR3','OR4'):
        ins = CELL_OUTPUT_PORTS[base][0]
        co_y = co('Y')
        for p in ins:
            others = [q for q in ins if q != p]
            result[p] = clamp(co_y * prod(cc0(q) for q in others))

    elif base in ('NAND2','NAND3','NAND4'):
        ins = CELL_OUTPUT_PORTS[base][0]
        co_y = co('Y')
        for p in ins:
            others = [q for q in ins if q != p]
            result[p] = clamp(co_y * prod(cc1(q) for q in others))

    elif base in ('NOR2','NOR3','NOR4'):
        ins = CELL_OUTPUT_PORTS[base][0]
        co_y = co('Y')
        for p in ins:
            others = [q for q in ins if q != p]
            result[p] = clamp(co_y * prod(cc0(q) for q in others))

    elif base == 'INV':
        result['A'] = clamp(co('Y'))

    elif base == 'BUF':
        result['A'] = clamp(co('Y'))

    elif base == 'AOI21':
        # Y = ~((A1&A2)|A3)
        # Approximate: treat as NOR( AND(A1,A2), A3 )
        and12 = cc1('A1') * cc1('A2')
        co_y  = co('Y')
        # Observability through AND-OR structure (approximation)
        result['A1'] = clamp(co_y * cc1('A2') * cc0('A3'))
        result['A2'] = clamp(co_y * cc1('A1') * cc0('A3'))
        result['A3'] = clamp(co_y * (1.0 - and12))

    elif base == 'AOI22':
        and1 = cc1('A1') * cc1('A2')
        and2 = cc1('A3') * cc1('A4')
        co_y = co('Y')
        result['A1'] = clamp(co_y * cc1('A2') * (1.0 - and2))
        result['A2'] = clamp(co_y * cc1('A1') * (1.0 - and2))
        result['A3'] = clamp(co_y * cc1('A4') * (1.0 - and1))
        result['A4'] = clamp(co_y * cc1('A3') * (1.0 - and1))

    elif base == 'OAI21':
        # Y = ~((A1|A2)&A3)
        or12 = 1.0 - cc0('A1') * cc0('A2')
        co_y = co('Y')
        result['A1'] = clamp(co_y * cc0('A2') * cc1('A3'))
        result['A2'] = clamp(co_y * cc0('A1') * cc1('A3'))
        result['A3'] = clamp(co_y * or12)

    elif base == 'OAI22':
        or1 = 1.0 - cc0('A1') * cc0('A2')
        or2 = 1.0 - cc0('A3') * cc0('A4')
        co_y = co('Y')
        result['A1'] = clamp(co_y * cc0('A2') * or2)
        result['A2'] = clamp(co_y * cc0('A1') * or2)
        result['A3'] = clamp(co_y * cc0('A4') * or1)
        result['A4'] = clamp(co_y * cc0('A3') * or1)

    elif base == 'OAI221':
        or1 = 1.0 - cc0('A1') * cc0('A2')
        or2 = 1.0 - cc0('A3') * cc0('A4')
        co_y = co('Y')
        result['A1'] = clamp(co_y * cc0('A2') * or2 * cc1('A5'))
        result['A2'] = clamp(co_y * cc0('A1') * or2 * cc1('A5'))
        result['A3'] = clamp(co_y * cc0('A4') * or1 * cc1('A5'))
        result['A4'] = clamp(co_y * cc0('A3') * or1 * cc1('A5'))
        result['A5'] = clamp(co_y * or1 * or2)

    elif base == 'AO21':
        co_y = co('Y')
        result['A1'] = clamp(co_y * cc1('A2') * cc0('A3'))
        result['A2'] = clamp(co_y * cc1('A1') * cc0('A3'))
        result['A3'] = clamp(co_y * (1.0 - cc1('A1')*cc1('A2')))

    elif base == 'AO22':
        and1 = cc1('A1') * cc1('A2')
        and2 = cc1('A3') * cc1('A4')
        co_y = co('Y')
        result['A1'] = clamp(co_y * cc1('A2') * (1.0-and2))
        result['A2'] = clamp(co_y * cc1('A1') * (1.0-and2))
        result['A3'] = clamp(co_y * cc1('A4') * (1.0-and1))
        result['A4'] = clamp(co_y * cc1('A3') * (1.0-and1))

    elif base == 'AO221':
        and1 = cc1('A1') * cc1('A2')
        and2 = cc1('A3') * cc1('A4')
        co_y = co('Y')
        result['A1'] = clamp(co_y * cc1('A2') * (1.0-and2) * cc0('A5'))
        result['A2'] = clamp(co_y * cc1('A1') * (1.0-and2) * cc0('A5'))
        result['A3'] = clamp(co_y * cc1('A4') * (1.0-and1) * cc0('A5'))
        result['A4'] = clamp(co_y * cc1('A3') * (1.0-and1) * cc0('A5'))
        result['A5'] = clamp(co_y * (1.0 - (1.0-and1)*(1.0-and2)))

    elif base == 'AO222':
        and1 = cc1('A1') * cc1('A2')
        and2 = cc1('A3') * cc1('A4')
        and3 = cc1('A5') * cc1('A6')
        co_y = co('Y')
        result['A1'] = clamp(co_y * cc1('A2') * (1.0-and2) * (1.0-and3))
        result['A2'] = clamp(co_y * cc1('A1') * (1.0-and2) * (1.0-and3))
        result['A3'] = clamp(co_y * cc1('A4') * (1.0-and1) * (1.0-and3))
        result['A4'] = clamp(co_y * cc1('A3') * (1.0-and1) * (1.0-and3))
        result['A5'] = clamp(co_y * cc1('A6') * (1.0-and1) * (1.0-and2))
        result['A6'] = clamp(co_y * cc1('A5') * (1.0-and1) * (1.0-and2))

    elif base == 'OA21':
        co_y = co('Y')
        result['A1'] = clamp(co_y * cc0('A2') * cc1('A3'))
        result['A2'] = clamp(co_y * cc0('A1') * cc1('A3'))
        result['A3'] = clamp(co_y * (1.0 - cc0('A1')*cc0('A2')))

    elif base == 'OA22':
        or1 = 1.0 - cc0('A1') * cc0('A2')
        or2 = 1.0 - cc0('A3') * cc0('A4')
        co_y = co('Y')
        result['A1'] = clamp(co_y * cc0('A2') * or2)
        result['A2'] = clamp(co_y * cc0('A1') * or2)
        result['A3'] = clamp(co_y * cc0('A4') * or1)
        result['A4'] = clamp(co_y * cc0('A3') * or1)

    elif base == 'OA221':
        or1 = 1.0 - cc0('A1') * cc0('A2')
        or2 = 1.0 - cc0('A3') * cc0('A4')
        co_y = co('Y')
        result['A1'] = clamp(co_y * cc0('A2') * or2 * cc1('A5'))
        result['A2'] = clamp(co_y * cc0('A1') * or2 * cc1('A5'))
        result['A3'] = clamp(co_y * cc0('A4') * or1 * cc1('A5'))
        result['A4'] = clamp(co_y * cc0('A3') * or1 * cc1('A5'))
        result['A5'] = clamp(co_y * or1 * or2)

    elif base == 'OA222':
        or1 = 1.0 - cc0('A1') * cc0('A2')
        or2 = 1.0 - cc0('A3') * cc0('A4')
        or3 = 1.0 - cc0('A5') * cc0('A6')
        co_y = co('Y')
        result['A1'] = clamp(co_y * cc0('A2') * or2 * or3)
        result['A2'] = clamp(co_y * cc0('A1') * or2 * or3)
        result['A3'] = clamp(co_y * cc0('A4') * or1 * or3)
        result['A4'] = clamp(co_y * cc0('A3') * or1 * or3)
        result['A5'] = clamp(co_y * cc0('A6') * or1 * or2)
        result['A6'] = clamp(co_y * cc0('A5') * or1 * or2)

    elif base == 'MUX21':
        co_y = co('Y')
        result['A1'] = clamp(co_y * cc0('S0'))
        result['A2'] = clamp(co_y * cc1('S0'))
        result['S0'] = clamp(co_y * abs(cc1('A1') - cc1('A2')))

    elif base == 'MUX41':
        co_y = co('Y')
        for p in ['A1','A2','A3','A4']:
            result[p] = clamp(co_y * 0.25)
        result['S0'] = clamp(co_y * 0.5)
        result['S1'] = clamp(co_y * 0.5)

    elif base == 'HADD':
        # SO = A0 XOR B0,  CO = A0 AND B0
        co_so = co('SO')
        co_co = co('CO')
        result['A0'] = clamp(co_so * cc0('B0') + co_so * cc1('B0') + co_co * cc1('B0'))
        result['B0'] = clamp(co_so * cc0('A0') + co_so * cc1('A0') + co_co * cc1('A0'))

    elif base == 'FADD':
        co_s  = co('S')
        co_co = co('CO')
        # Approximate
        result['A']  = clamp(co_s * 1.0 + co_co * (cc1('B') + cc1('CI') - cc1('B')*cc1('CI')))
        result['B']  = clamp(co_s * 1.0 + co_co * (cc1('A') + cc1('CI') - cc1('A')*cc1('CI')))
        result['CI'] = clamp(co_s * 1.0 + co_co * (cc1('A') + cc1('B')  - cc1('A')*cc1('B')))
        # Clamp again
        for k in result:
            result[k] = clamp(result[k])

    elif base in ('SDFFX2', 'DFFX1'):
        # D input observability = co(Q)
        co_q  = co('Q')
        co_qn = co('QN')
        result['D'] = clamp(co_q + co_qn)

    else:
        for ip in CELL_OUTPUT_PORTS.get(base, (['A'], ['Y']))[0]:
            result[ip] = 0.0

    return result


# ---------------------------------------------------------------------------
# 4.  TOPOLOGICAL SORT  (Kahn's algorithm on the net/gate graph)
# ---------------------------------------------------------------------------

def build_and_sort(ports_in, ports_out, instances, assigns):
    """
    Build a net-level graph and return instances in topological order
    (PI-to-PO direction).

    net_driver[net]  -> instance index that drives it (or None for PI)
    net_fanout[net]  -> list of instance indices that read it
    """
    n = len(instances)

    # Map net -> driving instance index
    net_driver = {}   # net -> inst_idx  (None = PI)
    net_fanout = defaultdict(list)  # net -> [inst_idx, ...]

    # PIs drive themselves
    for pi in ports_in:
        net_driver[pi] = None  # driven by PI

    # Assigns: lhs driven by rhs (treat as a wire alias)
    assign_map = {}  # lhs -> rhs
    for lhs, rhs in assigns:
        assign_map[lhs] = rhs

    def resolve(net):
        """Follow assign chain to find ultimate driver."""
        visited = set()
        while net in assign_map and net not in visited:
            visited.add(net)
            net = assign_map[net]
        return net

    # For each instance, determine which nets it drives (outputs)
    # and which nets it reads (inputs)
    inst_outputs = []   # inst_idx -> list of driven nets
    inst_inputs  = []   # inst_idx -> list of read nets

    for idx, inst in enumerate(instances):
        cell  = inst['cell']
        conn  = inst['conn']
        base  = get_cell_base(cell)
        info  = CELL_OUTPUT_PORTS.get(base)

        if info is None:
            unknown_cells.add(base)
            continue

        in_ports, out_ports = info

        outs = []
        for op in out_ports:
            net = conn.get(op)
            if net:
                net_driver[net] = idx
                outs.append(net)
        inst_outputs.append(outs)

        ins = []
        for ip in in_ports:
            net = conn.get(ip)
            if net:
                rnet = resolve(net)
                net_fanout[rnet].append(idx)
                ins.append(rnet)
        inst_inputs.append(ins)

    # Kahn topological sort
    # in-degree of each instance = number of its input nets whose
    # driver has not yet been processed
    in_degree = [0] * n
    for idx in range(n):
        for net in inst_inputs[idx]:
            drv = net_driver.get(net)
            if drv is not None:  # driven by another gate
                in_degree[idx] += 1

    # A gate is ready when all its input nets are resolved
    # Reframe: build gate->gate dependency
    gate_deps = [set() for _ in range(n)]   # gate_deps[i] = set of gates i depends on
    for idx in range(n):
        for net in inst_inputs[idx]:
            drv = net_driver.get(net)
            if drv is not None:
                gate_deps[idx].add(drv)

    in_deg = [len(gate_deps[i]) for i in range(n)]
    gate_fanout = defaultdict(list)  # gate -> gates that depend on it
    for idx in range(n):
        for dep in gate_deps[idx]:
            gate_fanout[dep].append(idx)

    queue = deque(i for i in range(n) if in_deg[i] == 0)
    topo  = []
    while queue:
        i = queue.popleft()
        topo.append(i)
        for j in gate_fanout[i]:
            in_deg[j] -= 1
            if in_deg[j] == 0:
                queue.append(j)

    return topo, net_driver, net_fanout, inst_inputs, inst_outputs, assign_map


# ---------------------------------------------------------------------------
# 5.  COP MAIN PASS
# ---------------------------------------------------------------------------

def run_cop(ports_in, ports_out, instances, assigns):
    """
    Run the COP algorithm.
    Returns:
        cc1[net]  : 1-controllability  (float in [0,1])
        co[net]   : observability      (float in [0,1])
    """
    topo, net_driver, net_fanout, inst_inputs, inst_outputs, assign_map = \
        build_and_sort(ports_in, ports_out, instances, assigns)

    # --- FORWARD PASS: compute cc1 for every net ---
    cc1 = {}

    # Initialize PIs to 0.5
    for pi in ports_in:
        cc1[pi] = 0.5

    # Process gates in topological order
    for idx in topo:
        inst  = instances[idx]
        cell  = inst['cell']
        conn  = inst['conn']
        base  = get_cell_base(cell)
        info  = CELL_OUTPUT_PORTS.get(base)
        if info is None:
            continue

        in_ports, out_ports = info

        # Gather input cc1 values (resolve assigns)
        input_cc1 = {}
        for ip in in_ports:
            net = conn.get(ip)
            if net:
                rnet = net
                visited = set()
                while rnet in assign_map and rnet not in visited:
                    visited.add(rnet)
                    rnet = assign_map[rnet]
                input_cc1[ip] = cc1.get(rnet, 0.5)

        out_cc1 = compute_cc1(base, input_cc1, conn)

        for op, val in out_cc1.items():
            net = conn.get(op)
            if net:
                cc1[net] = val

    # Propagate cc1 through assigns
    for lhs, rhs in assigns:
        if rhs in cc1 and lhs not in cc1:
            cc1[lhs] = cc1[rhs]

    # --- BACKWARD PASS: compute observability for every net ---
    co = {}

    # POs have observability = 1.0
    for po in ports_out:
        co[po] = 1.0

    # Scan FF outputs (Q, QN) are observable through scan chain
    for inst in instances:
        base = get_cell_base(inst['cell'])
        if base in ('SDFFX2', 'DFFX1'):
            for op in ['Q', 'QN']:
                net = inst['conn'].get(op)
                if net:
                    co[net] = max(co.get(net, 0.0), 1.0)

    # Process gates in REVERSE topological order
    for idx in reversed(topo):
        inst  = instances[idx]
        cell  = inst['cell']
        conn  = inst['conn']
        base  = get_cell_base(cell)
        info  = CELL_OUTPUT_PORTS.get(base)
        if info is None:
            continue

        in_ports, out_ports = info

        # Gather input cc1 values (needed for co computation)
        input_cc1 = {}
        for ip in in_ports:
            net = conn.get(ip)
            if net:
                rnet = net
                visited = set()
                while rnet in assign_map and rnet not in visited:
                    visited.add(rnet)
                    rnet = assign_map[rnet]
                input_cc1[ip] = cc1.get(rnet, 0.5)

        # Gather output co values
        co_outputs = {}
        for op in out_ports:
            net = conn.get(op)
            if net:
                co_outputs[op] = co.get(net, 0.0)

        # Compute input observabilities
        co_inputs = compute_co_inputs(base, input_cc1, co_outputs)

        # Accumulate: a net may be read by multiple gates
        for ip, val in co_inputs.items():
            net = conn.get(ip)
            if net:
                rnet = net
                visited = set()
                while rnet in assign_map and rnet not in visited:
                    visited.add(rnet)
                    rnet = assign_map[rnet]
                # Observability of a net with multiple fanouts:
                # co(net) = 1 - prod(1 - co_i) for each fanout path
                # Approximate: use max (common simplification)
                co[rnet] = clamp(max(co.get(rnet, 0.0), val))

    # Propagate co through assigns (backward)
    for lhs, rhs in reversed(assigns):
        if lhs in co and rhs not in co:
            co[rhs] = co[lhs]

    return cc1, co


# ---------------------------------------------------------------------------
# 6.  TP FILE PARSER & CANDIDATE IDENTIFICATION
# ---------------------------------------------------------------------------

def parse_tp_file(tp_file):
    """
    Parse TetraMAX tp_analysis.txt.
    Returns:
        cps: list of (net, type)  type in {'control_0','control_1'}
        ops: list of net
    """
    cps = []
    ops = []
    with open(tp_file, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            # continuation lines end with backslash in original but we
            # deal with them by joining in a pre-pass
            m = re.match(r'set_test_point_element\s*\{([^}]+)\}\s*-type\s+(\S+)', line)
            if m:
                nets_raw = m.group(1).split()
                tp_type  = m.group(2)
                nets = [n.strip('\\') for n in nets_raw if n.strip('\\')]
                if tp_type in ('control_0', 'control_1'):
                    for net in nets:
                        cps.append((net, tp_type))
                elif tp_type == 'observe':
                    for net in nets:
                        ops.append(net)
    return cps, ops


def identify_candidates(cps, ops, cc1, co, verbose=True):
    """
    Apply SPAR Section IV thresholds to classify EO-CPs and EC-OPs.

    CCTh = max( avg(CC of OR-CPs), avg(1 - CC of AND-CPs) )
    COTh = avg(CO of OPs)
    """
    # Compute average CC of CP types
    or_cps  = [net for net, t in cps if t == 'control_1']
    and_cps = [net for net, t in cps if t == 'control_0']

    def net_cc1(net):
        # Try direct, then with /Y suffix stripped
        return cc1.get(net, cc1.get(net.rstrip('/Y'), 0.5))

    def net_co(net):
        return co.get(net, co.get(net.rstrip('/Y'), 0.0))

    avg_cc_or  = (sum(net_cc1(n) for n in or_cps)  / len(or_cps))  if or_cps  else 0.0
    avg_1mcc_and = (sum(1.0 - net_cc1(n) for n in and_cps) / len(and_cps)) if and_cps else 0.0
    CC_Th = max(avg_cc_or, avg_1mcc_and)

    avg_co_op = (sum(net_co(n) for n in ops) / len(ops)) if ops else 0.0
    CO_Th = avg_co_op

    if verbose:
        print("\n--- COP-based Thresholds ---")
        print("  CCTh = max(avg CC of OR-CPs={:.4f}, avg (1-CC) of AND-CPs={:.4f}) = {:.4f}".format(
            avg_cc_or, avg_1mcc_and, CC_Th))
        print("  COTh = avg CO of OPs = {:.4f}".format(CO_Th))

    # Identify EO-CPs: CP lines with CO > COTh
    eo_cps = []
    for net, tp_type in cps:
        obs = net_co(net)
        if obs > CO_Th:
            eo_cps.append((net, tp_type, obs))

    # Identify EC-OPs: OP lines where CCTh < CC < 1-CCTh
    ec_ops = []
    for net in ops:
        ctrl = net_cc1(net)
        if CC_Th < ctrl < (1.0 - CC_Th):
            ec_ops.append((net, ctrl))

    return eo_cps, ec_ops, CC_Th, CO_Th


# ---------------------------------------------------------------------------
# 7.  MAIN
# ---------------------------------------------------------------------------

def main():
    import argparse
    parser = argparse.ArgumentParser(description='COP testability analysis for SPAR')
    parser.add_argument('netlist', help='Gate-level Verilog netlist')
    parser.add_argument('--tp_file', help='TetraMAX tp_analysis.txt', default=None)
    args = parser.parse_args()

    print("Parsing netlist: {}".format(args.netlist))
    ports_in, ports_out, instances, assigns = parse_verilog(args.netlist)
    # Build instance/pin -> net lookup
    pin_to_net = {}

    for inst in instances:
        inst_name = inst['inst']

        for port, net in inst['conn'].items():
            pin_to_net["{}/{}".format(inst_name, port)] = net
    
    print("  PIs={}, POs={}, instances={}, assigns={}".format(
        len(ports_in), len(ports_out), len(instances), len(assigns)))

    print("Running COP...")
    cc1_vals, co_vals = run_cop(ports_in, ports_out, instances, assigns)
    print("  CC1 computed for {} nets".format(len(cc1_vals)))
    print("  CO  computed for {} nets".format(len(co_vals)))
    print("\nUnknown cell types:")
    for c in sorted(unknown_cells):
        print(" ", c)

    # Print some sample values
    print("\n--- Sample PI controllabilities (should all be ~0.5) ---")
    for pi in sorted(list(ports_in))[:5]:
        print("  PI {:20s}  CC1={:.4f}  CO={:.4f}".format(
            pi, cc1_vals.get(pi, -1), co_vals.get(pi, -1)))

    print("\n--- Sample PO observabilities (should all be 1.0) ---")
    for po in sorted(list(ports_out))[:5]:
        print("  PO {:20s}  CC1={:.4f}  CO={:.4f}".format(
            po, cc1_vals.get(po, -1), co_vals.get(po, -1)))

    if args.tp_file:
        print("\nParsing TP file: {}".format(args.tp_file))
        cps, ops = parse_tp_file(args.tp_file)
        print("  CPs={}, OPs={}".format(len(cps), len(ops)))

        for net, tp_type in cps:
            if net not in cc1_vals:
                print("NOT FOUND:", net)

        print("\n--- CC1 and CO for all CPs ---")
        for net, tp_type in cps:

            real_net = pin_to_net.get(net, net)
                print("{} -> {}".format(net, real_net))
                ctrl = cc1_vals.get(real_net, -1)
                obs  = co_vals.get(real_net, -1)
            
            cp_label = 'OR-CP' if tp_type == 'control_1' else 'AND-CP'
            print("  {:8s}  {:30s}  CC1={:.4f}  CO={:.6f}".format(
                cp_label, net, ctrl, obs))

        print("\n--- CC1 and CO for all OPs ---")
        for net in ops:
            real_net = pin_to_net.get(net, net)

            ctrl = cc1_vals.get(real_net, -1)
            obs  = co_vals.get(real_net, -1)
            print("  OP  {:30s}  CC1={:.4f}  CO={:.6f}".format(net, ctrl, obs))

        eo_cps, ec_ops, CC_Th, CO_Th = identify_candidates(
            cps, ops, cc1_vals, co_vals)

        print("\n--- EO-CP candidates (CO > COTh={:.4f}) ---".format(CO_Th))
        for net, tp_type, obs in eo_cps:
            cp_label = 'OR-CP' if tp_type == 'control_1' else 'AND-CP'
            print("  {:8s}  {:30s}  CO={:.6f}".format(cp_label, net, obs))

        print("\n--- EC-OP candidates (CCTh={:.4f} < CC1 < {:.4f}) ---".format(
            CC_Th, 1.0-CC_Th))
        for net, ctrl in ec_ops:
            print("  OP  {:30s}  CC1={:.4f}".format(net, ctrl))

        print("\nSummary: EO-CPs={}, EC-OPs={}".format(len(eo_cps), len(ec_ops)))


if __name__ == '__main__':
    main()
