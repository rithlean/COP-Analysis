#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
SPAR: Shared Point Insertion for Area Overhead Reduction
Kim et al., IEEE TCAD Vol.41 No.11, 2022

Fully autonomous flow:
  - Parses gate-level scan netlist (DC output)
  - Computes COP controllability + observability
  - Identifies CP/OP sites autonomously (no external tool needed)
  - Runs cone analysis (Rule 1 + Rule 2)
  - Runs controllability optimisation (Eq. 7/8)
  - Emits complete modified Verilog netlist (DC-ready)

Usage:
  python 03_spar.py netlist/scan_netlist_flat.v [DTh] [cp_budget] [op_budget]

  DTh        : difference threshold 0.0-1.0  (default 1.0, paper setting)
  cp_budget  : max CPs as fraction of scan cells (default 0.05 = 5%)
  op_budget  : max OPs as fraction of scan cells (default 0.05 = 5%)
"""

from __future__ import print_function, division

import re, json, sys, math, os
from collections import defaultdict, deque

# ===============================================================
#  SECTION 1 - DATA STRUCTURES
# ===============================================================

class Gate(object):
    __slots__ = ('gtype','name','output','inputs')
    def __init__(self, gtype, name, output, inputs):
        self.gtype   = gtype    # canonical type string
        self.name    = name
        self.output  = output   # single output net name
        self.inputs  = inputs   # list[str] of input net names

class Netlist(object):
    def __init__(self):
        self.gates    = {}                   # name  -> Gate
        self.net_src  = {}                   # net   -> Gate (driver)
        self.net_dst  = defaultdict(list)    # net   -> [Gate] (sinks)
        self.pis      = []                   # primary input net names
        self.pos      = []                   # primary output net names
        self.scan_ffs = []                   # scan FF output nets (PPIs)
        self.scan_din = []                   # scan FF D-input nets (PPOs)
        self.all_nets = set()

    # -- Traversal helpers --------------------------------------

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


# ===============================================================
#  SECTION 2 - GATE-LEVEL VERILOG PARSER
# ===============================================================

CELL_TYPE_MAP = [
    ('SDFFARX',  'DFF'),
    ('SDFF',     'DFF'),
    ('DFF',      'DFF'),
    ('HADD',     'XOR'),
    ('AOI222',   'NAND'),
    ('AOI22',    'NAND'),
    ('AOI21',    'NAND'),
    ('AO222',    'AND'),
    ('AO221',    'AND'),
    ('AO22',     'AND'),
    ('AO21',     'AND'),
    ('OA221',    'OR'),
    ('OA21',     'OR'),
    ('XNOR',     'XNOR'),
    ('XOR3',     'XOR'),
    ('XOR2',     'XOR'),
    ('XOR',      'XOR'),
    ('NAND',     'NAND'),
    ('NOR',      'NOR'),
    ('AND',      'AND'),
    ('OR',       'OR'),
    ('INVX',     'NOT'),
    ('NBUFF',    'BUF'),
    ('BUF',      'BUF'),
    ('MUX21',    'MUX'),
]

OUTPUT_PORTS_ORDERED = ['Q', 'QN', 'Y', 'SO', 'C1']
OUTPUT_PORTS = set(OUTPUT_PORTS_ORDERED)


def canonical_type(cell_name):
    upper = cell_name.upper()
    for pattern, ctype in CELL_TYPE_MAP:
        if pattern in upper:
            return ctype
    return 'UNKNOWN'


def parse_port_list(port_str):
    """
    Parse .port(net) style connections into dict.
    Handles plain names, bus names, and DC escaped identifiers.
    """
    ports = {}
    net_pat = r'(\\[\w\[\]./]+ *|[\w]+(?:\[\d+\])?)'
    pat = re.compile(r'\.(\w+)\s*\(\s*' + net_pat + r'\s*\)')
    for m in pat.finditer(port_str):
        port = m.group(1)
        net  = m.group(2).strip()
        ports[port] = net
    return ports


def parse_verilog(filepath):
    """
    Gate-level Verilog parser for DC-generated netlists.
    Preserves the original raw text for regeneration.
    """
    nl = Netlist()

    with open(filepath) as f:
        raw = f.read()

    # Store original text for regeneration
    nl.original_text = raw

    # Strip comments for parsing only
    stripped = re.sub(r'//[^\n]*', '', raw)
    stripped = re.sub(r'/\*.*?\*/', '', stripped, flags=re.DOTALL)

    modules = list(re.finditer(
        r'\bmodule\s+(\w+)\s*(?:\([^)]*\))?\s*;(.*?)endmodule',
        stripped, re.DOTALL))

    if not modules:
        raise ValueError("No module found in {0}".format(filepath))

    mod_match  = modules[-1]
    nl.mod_name = mod_match.group(1)
    mod_body   = mod_match.group(2)

    # Store module body start/end positions in stripped text for regeneration
    nl.mod_body_start = mod_match.start(2)
    nl.mod_body_end   = mod_match.end(2)

    def extract_signals(keyword, body):
        sigs = []
        for m in re.finditer(
                r'\b' + keyword + r'\b\s*(?:\[\s*\d+\s*:\s*\d+\s*\])?\s*([\w\s,]+?)\s*;',
                body):
            for sig in re.split(r'[\s,]+', m.group(1)):
                sig = sig.strip()
                if sig:
                    sigs.append(sig)
        return sigs

    nl.pis = extract_signals('input',  mod_body)
    nl.pos = extract_signals('output', mod_body)

    inst_re = re.compile(
        r'(\w+)\s+(\\[^\s(]+|\w+)\s*\(([^;]*?)\)\s*;', re.DOTALL)

    NON_DATA_PORTS = {'CLK', 'CK', 'RSTB', 'RST', 'RB', 'RN',
                      'SE', 'SI', 'SIN', 'TE', 'TI'}

    for m in inst_re.finditer(mod_body):
        cell_name = m.group(1)
        inst_name = m.group(2)
        port_str  = m.group(3)

        if cell_name in ('module','input','output','wire',
                         'reg','assign','always','if','else'):
            continue

        ctype = canonical_type(cell_name)
        if ctype == 'UNKNOWN':
            continue

        ports = parse_port_list(port_str)
        if 'S0' in ports:
            ports['SEL'] = ports.pop('S0')
        if 'A1' in ports and 'A2' in ports and 'SEL' in ports:
            ports['D0'] = ports.pop('A1')
            ports['D1'] = ports.pop('A2')

        if not ports:
            continue

        out_net = None
        for op in OUTPUT_PORTS_ORDERED:
            if op in ports and ports[op]:
                out_net = ports[op]
                break

        if out_net is None:
            vals = list(ports.values())
            if vals:
                out_net = vals[-1]
            else:
                continue

        inp_nets = [v for k, v in ports.items()
                    if k not in OUTPUT_PORTS
                    and k not in NON_DATA_PORTS
                    and v and v != out_net]

        g = Gate(ctype, inst_name, out_net, inp_nets)
        nl.gates[inst_name] = g
        nl.net_src[out_net] = g
        for inp in inp_nets:
            nl.net_dst[inp].append(g)
        nl.all_nets.add(out_net)
        nl.all_nets.update(inp_nets)

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

        if ctype == 'DFF':
            nl.scan_ffs.append(out_net)
            d_net = ports.get('D') or ports.get('d')
            if d_net and d_net in inp_nets:
                nl.scan_din.append(d_net)
            elif inp_nets:
                nl.scan_din.append(inp_nets[0])

    print("  Parsed: {0} gates, {1} PIs, {2} scan FFs".format(
        len(nl.gates), len(nl.pis), len(nl.scan_ffs)))
    return nl


# ===============================================================
#  SECTION 3 - COP TESTABILITY
# ===============================================================

def compute_cop(nl):
    CC = {}
    CO = {}

    for pi  in nl.pis:      CC[pi]  = 0.5
    for ppi in nl.scan_ffs: CC[ppi] = 0.5
    for po  in nl.pos:      CO[po]  = 1.0
    for ppo in nl.scan_din: CO[ppo] = 1.0

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
                if len(ins) >= 3:
                    d0, d1, sel = ins[0], ins[1], ins[2]
                    cc = sel * d1 + (1.0 - sel) * d0
                else:
                    cc = 0.5
            elif t == 'DFF':
                cc = 0.5
            else:
                cc = 0.5

            cc = max(1e-9, min(1.0 - 1e-9, cc))
            if abs(CC.get(g.output, -1) - cc) > 1e-10:
                CC[g.output] = cc
                changed = True

        if not changed:
            break

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
                    sens = 1.0
                    for v in others: sens *= v
                elif t in ('OR', 'NOR'):
                    sens = 1.0
                    for v in others: sens *= (1.0 - v)
                elif t in ('NOT', 'BUF'):
                    sens = 1.0
                elif t in ('XOR', 'XNOR'):
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
                    sens = 1.0
                else:
                    sens = 0.5

                new_co = co_out * sens
                if new_co > CO.get(inp, 0.0) + 1e-12:
                    CO[inp] = new_co
                    changed = True

        if not changed:
            break

    return CC, CO


# ===============================================================
#  SECTION 4 - AUTONOMOUS CP/OP SITE IDENTIFICATION
# ===============================================================

def identify_cp_op_sites(nl, CC, CO, cp_budget_frac=0.05,
                          op_budget_frac=0.05):
    n_scan   = max(len(nl.scan_ffs), 1)
    cp_limit = max(1, int(n_scan * cp_budget_frac))
    op_limit = max(1, int(n_scan * op_budget_frac))

    excluded = set(nl.pis) | set(nl.pos) | \
               set(nl.scan_ffs) | set(nl.scan_din)

    internal_nets = [n for n in nl.all_nets if n not in excluded]

    def cp_score(net):
        cc = CC.get(net, 0.5)
        return abs(cc - 0.5)

    cp_candidates = sorted(internal_nets, key=cp_score, reverse=True)

    cp_list = []
    for net in cp_candidates:
        if len(cp_list) >= cp_limit:
            break
        cc = CC.get(net, 0.5)
        if abs(cc - 0.5) < 0.05:
            continue
        cp_type = 'OR' if cc < 0.5 else 'AND'
        cp_list.append((net, cp_type))

    op_candidates = sorted(internal_nets,
                           key=lambda n: CO.get(n, 0.0))

    op_list = []
    for net in op_candidates:
        if len(op_list) >= op_limit:
            break
        co = CO.get(net, 0.0)
        if co > 0.1:
            break
        op_list.append(net)

    print("  CP sites identified : {0} (budget {1}, {2:.0f}% of {3} scan FFs)".format(
        len(cp_list), cp_limit, cp_budget_frac * 100, n_scan))
    print("  OP sites identified : {0} (budget {1})".format(
        len(op_list), op_limit))
    return cp_list, op_list


# ===============================================================
#  SECTION 5 - THRESHOLD COMPUTATION
# ===============================================================

def compute_thresholds(cp_list, op_list, CC, CO):
    or_cc  = [CC.get(n, 0.5)       for n, t in cp_list if t == 'OR']
    and_cc = [1.0 - CC.get(n, 0.5) for n, t in cp_list if t == 'AND']

    avg_or  = sum(or_cc)  / max(len(or_cc),  1)
    avg_and = sum(and_cc) / max(len(and_cc), 1)
    CCTh    = max(avg_or, avg_and)

    op_co  = [CO.get(n, 0.0) for n in op_list]
    COTh   = sum(op_co) / max(len(op_co), 1)

    return CCTh, COTh


# ===============================================================
#  SECTION 6 - CANDIDATE IDENTIFICATION
# ===============================================================

def identify_shared_candidates(cp_list, op_list, CC, CO,
                                CCTh, COTh):
    eo_cps = [(net, t) for net, t in cp_list
              if CO.get(net, 0.0) > COTh]

    ec_ops = [net for net in op_list
              if CCTh <= CC.get(net, 0.5) <= 1.0 - CCTh]

    print("  EO-CPs : {0} / {1}".format(len(eo_cps), len(cp_list)))
    print("  EC-OPs : {0} / {1}".format(len(ec_ops), len(op_list)))
    return eo_cps, ec_ops


# ===============================================================
#  SECTION 7 - CONE ANALYSIS
# ===============================================================

def check_rule1(nl, eo_cp_net, ec_op_net):
    fanout = nl.fanout_cone(eo_cp_net)
    return ec_op_net not in fanout


def check_rule2(nl, eo_cp_net, ec_op_net, cp_type, CC, direct):
    if cp_type == 'OR':
        ctrl_val = 1.0 if direct else 0.0
    else:
        ctrl_val = 0.0 if direct else 1.0

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

    direct_sinks = nl.net_dst.get(eo_cp_net, [])
    if not direct_sinks:
        return True

    any_unblocked = False
    for sink_gate in direct_sinks:
        if sink_gate.gtype == 'DFF':
            any_unblocked = True
            continue
        other_ins = [temp_CC.get(i, CC.get(i, 0.5))
                     for i in sink_gate.inputs
                     if i != eo_cp_net]
        t = sink_gate.gtype

        if t in ('AND', 'NAND'):
            sens = 1.0
            for v in other_ins: sens *= v
        elif t in ('OR', 'NOR'):
            sens = 1.0
            for v in other_ins: sens *= (1.0 - v)
        elif t in ('NOT', 'BUF', 'XOR', 'XNOR'):
            sens = 1.0
        else:
            sens = 0.5

        if sens > 1e-6:
            any_unblocked = True
            break

    return any_unblocked


# ===============================================================
#  SECTION 8 - CONTROLLABILITY OPTIMISATION
# ===============================================================

def compute_cc_req(nl, eo_cp_net, cp_type):
    fanout_total = len(nl.fanout_cone(eo_cp_net))
    direct_sinks = nl.net_dst.get(eo_cp_net, [])

    bx = 0
    Bx = 0

    for g in direct_sinks:
        if g.gtype == 'DFF':
            continue
        cone_size = len(nl.fanout_cone(g.output))
        if g.gtype in ('OR', 'NOR'):
            bx += cone_size
        elif g.gtype in ('AND', 'NAND'):
            Bx += cone_size

    Fx    = max(fanout_total - bx - Bx, 0)
    total = bx + Bx + Fx

    if total == 0:
        return 0.5

    if cp_type == 'OR':
        return Bx / float(total)
    else:
        return (Bx + Fx) / float(total)


def select_best_ec_op(candidate_nets, cc_req, CC, DTh=1.0):
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


# ===============================================================
#  SECTION 9 - MAIN SPAR FLOW
# ===============================================================

def run_spar(netlist_path, DTh=1.0, cp_budget=0.05, op_budget=0.05):

    print("\n" + "="*55)
    print(" SPAR: Shared Point Insertion for Area Overhead Reduction")
    print("="*55)

    print("\n[1/7] Parsing netlist ...")
    nl = parse_verilog(netlist_path)

    print("\n[2/7] COP testability analysis ...")
    CC, CO = compute_cop(nl)
    print("  CC computed for {0} nets".format(len(CC)))
    print("  CO computed for {0} nets".format(len(CO)))

    print("\n[3/7] Autonomous CP/OP site identification ...")
    cp_list, op_list = identify_cp_op_sites(nl, CC, CO, cp_budget, op_budget)

    print("\n[4/7] Computing EO-CP / EC-OP thresholds ...")
    CCTh, COTh = compute_thresholds(cp_list, op_list, CC, CO)
    print("  CCTh = {0:.6f}".format(CCTh))
    print("  COTh = {0:.6f}".format(COTh))

    print("\n[5/7] Identifying shared point candidates ...")
    eo_cps, ec_ops = identify_shared_candidates(
        cp_list, op_list, CC, CO, CCTh, COTh)

    print("\n[6/7] Pairing (cone analysis + ctrl optimisation) ...")

    shared_points     = []
    used_ec_ops       = set()
    conventional_cps  = []
    conventional_ops  = []

    for eo_cp_net, cp_type in eo_cps:
        cc_req = compute_cc_req(nl, eo_cp_net, cp_type)

        valid = []
        for ec_op_net in ec_ops:
            if ec_op_net in used_ec_ops:
                continue
            if not check_rule1(nl, eo_cp_net, ec_op_net):
                continue
            ok_direct   = check_rule2(nl, eo_cp_net, ec_op_net,
                                      cp_type, CC, direct=True)
            ok_inverted = check_rule2(nl, eo_cp_net, ec_op_net,
                                      cp_type, CC, direct=False)
            if ok_direct or ok_inverted:
                valid.append((ec_op_net, ok_direct, ok_inverted))

        if not valid:
            conventional_cps.append((eo_cp_net, cp_type))
            continue

        valid_nets = [n for n, od, oi in valid]
        best_net, use_inv = select_best_ec_op(valid_nets, cc_req, CC, DTh)

        if best_net is None:
            conventional_cps.append((eo_cp_net, cp_type))
            continue

        ok_d = next((od for n, od, oi in valid if n == best_net), False)
        ok_i = next((oi for n, od, oi in valid if n == best_net), False)

        if use_inv and not ok_i:
            use_inv = False
        elif not use_inv and not ok_d:
            use_inv = True

        if   cp_type == 'AND' and not use_inv: sp_type = 1
        elif cp_type == 'AND' and use_inv:     sp_type = 2
        elif cp_type == 'OR'  and not use_inv: sp_type = 3
        else:                                  sp_type = 4

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

        bridge = Gate('BUF',
                      '__spar_bridge_{0}'.format(len(shared_points)),
                      eo_cp_net,
                      [best_net])
        nl.net_dst[best_net].append(bridge)

    conventional_ops = [n for n in op_list if n not in used_ec_ops]

    print("\n[7/7] Results summary")
    print("  Shared points inserted   : {0}".format(len(shared_points)))
    print("  Conventional CPs remain  : {0}".format(len(conventional_cps)))
    print("  Conventional OPs remain  : {0}".format(len(conventional_ops)))

    sp_ratio = len(shared_points) / float(max(len(eo_cps), 1)) * 100
    print("  SP ratio (of EO-CPs)     : {0:.1f}%".format(sp_ratio))

    area_saving_est = len(shared_points) / \
                      float(max(len(cp_list) + len(op_list), 1)) * 100
    print("  Estimated area reduction : ~{0:.1f}% "
          "(of conventional TP logic)".format(area_saving_est))

    return shared_points, conventional_cps, conventional_ops, nl


# ===============================================================
#  SECTION 10 - VERILOG NET NAME HELPER
# ===============================================================

def vlog_net(name):
    """
    Return a Verilog-safe net reference.
    Plain identifiers (letters/digits/underscore) are returned as-is.
    Names with special characters (DC escaped identifiers like
    \\C25/DATA5_0) are wrapped with backslash + trailing space.
    """
    if re.match(r'^[A-Za-z_]\w*$', name):
        return name                      # plain name
    if re.match(r'^\w+(\[\d+\])?$', name):
        return name                      # bus bit, e.g. REG1[0]
    # Escaped identifier — ensure leading backslash and trailing space
    bare = name.lstrip('\\').rstrip()
    return '\\' + bare + ' '


# ===============================================================
#  SECTION 11 - FULL NETLIST REGENERATION
# ===============================================================

def generate_full_netlist(shared_points, conventional_cps,
                           conventional_ops, nl, netlist_path,
                           output_path):
    """
    Produce a complete, DC-ready modified Verilog netlist by:

    1. Reading the original netlist text verbatim.
    2. Adding TPEnable to the module port list.
    3. Adding TPEnable input declaration.
    4. Adding wire declarations for new SP nets.
    5. Replacing downstream uses of each EO-CP net with the
       corresponding _spN_out net inside gate port lists.
    6. Appending shared point + conventional TP gate instances
       before endmodule.

    Net renaming rule:
      - The DRIVING gate of an EO-CP net keeps its output unchanged
        (it still drives the original net name, which feeds _spN_gate).
      - All SINK gates (gates that take the EO-CP net as INPUT) have
        that input replaced with _spN_out.
    This is correct because the SP gate sits between the original
    driver and the downstream logic, intercepting the signal.
    """

    with open(netlist_path) as f:
        text = f.read()

    # ----------------------------------------------------------
    # Build the net-rename map: old_net -> new_net for each SP
    # Key insight: we only rename INPUT uses, not the output of
    # the driving gate.  We identify sink gate instance names to
    # avoid touching the driver.
    # ----------------------------------------------------------
    rename_map = {}   # eo_cp_net -> _spN_out name
    for i, sp in enumerate(shared_points):
        rename_map[sp['eo_cp_net']] = '_sp{0}_out'.format(i)

    # Collect driving gate instance names so we skip them
    driver_instances = set()
    for net in rename_map:
        drv = nl.net_src.get(net)
        if drv:
            driver_instances.add(drv.name)

    # ----------------------------------------------------------
    # Rename net references inside port connection lists.
    # Strategy: find every  .PORT( net )  occurrence and replace
    # the net name if it is in rename_map, but only when the
    # enclosing instance is NOT the driver of that net.
    # ----------------------------------------------------------

    # We'll do a single pass: split text into instance blocks,
    # process each, then reassemble.

    # Pattern: CellType  InstName  ( ...port-list... ) ;
    inst_full_re = re.compile(
        r'(\b\w+\s+(?:\\[^\s(]+|\w+)\s*\()([^;]*?)(\)\s*;)',
        re.DOTALL)

    # Port connection pattern inside a block
    port_net_re = re.compile(
        r'(\.\w+\s*\(\s*)(\\[\w\[\]./]+ *|[\w]+(?:\[\d+\])?)?(\s*\))')

    def rename_ports_in_block(inst_text, inst_name):
        """Replace EO-CP net references with _spN_out in one instance."""
        if inst_name in driver_instances:
            return inst_text   # leave driver output untouched

        def replace_net(m):
            prefix = m.group(1)
            net    = m.group(2)
            suffix = m.group(3)
            if net is None:
                return m.group(0)
            net_stripped = net.strip()
            if net_stripped in rename_map:
                new_net = rename_map[net_stripped]
                return prefix + new_net + suffix
            return m.group(0)

        return port_net_re.sub(replace_net, inst_text)

    # Extract instance name from matched text
    inst_name_re = re.compile(r'^\s*\w+\s+(\\[^\s(]+|\w+)\s*\(')

    def process_match(m):
        full   = m.group(0)
        header = m.group(1)   # "CellType InstName ("
        ports  = m.group(2)   # port list
        tail   = m.group(3)   # ") ;"

        nm = inst_name_re.match(header)
        if nm is None:
            return full
        inst_name = nm.group(1).lstrip('\\').rstrip()

        new_ports = rename_ports_in_block(ports, inst_name)
        return header + new_ports + tail

    text = inst_full_re.sub(process_match, text)

    # ----------------------------------------------------------
    # Also rename any assign statements that reference an EO-CP net
    # e.g.  assign SO = n595;
    # ----------------------------------------------------------
    for old_net, new_net in rename_map.items():
        # Only rename right-hand side of assign statements
        assign_re = re.compile(
            r'(assign\s+\S+\s*=\s*)(' + re.escape(old_net) + r')(\s*;)')
        text = assign_re.sub(
            lambda m, nn=new_net: m.group(1) + nn + m.group(3), text)

    # ----------------------------------------------------------
    # 1. Add TPEnable to module port list
    #    Find:  module NAME ( ... );
    #    Append TPEnable before the closing )
    # ----------------------------------------------------------
    def add_tpenable_to_portlist(t):
        mod_re = re.compile(
            r'(module\s+\w+\s*\()([^)]*?)(\)\s*;)', re.DOTALL)
        def replacer(m):
            ports_str = m.group(2).rstrip()
            if 'TPEnable' in ports_str:
                return m.group(0)   # already there
            # Add on a new line aligned with DC style
            return m.group(1) + ports_str + ',\n        TPEnable ' + m.group(3)
        return mod_re.sub(replacer, t, count=1)

    text = add_tpenable_to_portlist(text)

    # ----------------------------------------------------------
    # 2. Add  input TPEnable;  after the last existing input decl
    # ----------------------------------------------------------
    if 'TPEnable' not in text.split('endmodule')[0].split('input')[-1]:
        last_input_re = re.compile(r'(input\b[^;]*;)', re.DOTALL)
        matches = list(last_input_re.finditer(text))
        if matches:
            last_m = matches[-1]
            insert_pos = last_m.end()
            text = text[:insert_pos] + '\n  input TPEnable;' + text[insert_pos:]

    # ----------------------------------------------------------
    # 3. Add wire declarations for new SP nets
    #    Insert after the last  wire ...;  block
    # ----------------------------------------------------------
    new_wires = []
    for i, sp in enumerate(shared_points):
        new_wires.append('_sp{0}_ctrl'.format(i))
        new_wires.append('_sp{0}_out'.format(i))
    for net, cp_type in conventional_cps:
        safe = re.sub(r'[^A-Za-z0-9_]', '_', net)
        new_wires.append('_conv_cp_{0}_out'.format(safe))

    if new_wires:
        wire_decl = '  wire   ' + ', '.join(new_wires) + ';'
        # Find last wire declaration block and insert after it
        wire_re = re.compile(r'(wire\b[^;]*;)', re.DOTALL)
        wire_matches = list(wire_re.finditer(text))
        if wire_matches:
            last_wire = wire_matches[-1]
            insert_pos = last_wire.end()
            text = text[:insert_pos] + '\n' + wire_decl + text[insert_pos:]
        else:
            # No wire block found — insert before first gate instance
            text = text.replace('endmodule',
                                 wire_decl + '\n\nendmodule', 1)

    # ----------------------------------------------------------
    # 4. Build shared point gate instances and insert before endmodule
    # ----------------------------------------------------------
    sp_lines = [
        '',
        '  // -------------------------------------------------------',
        '  // SPAR shared-point logic -- auto-generated',
        '  // TPEnable: 1 = test mode, 0 = functional mode',
        '  // -------------------------------------------------------',
        '',
    ]

    # SP gate templates (positional primitive syntax, DC-compatible)
    #
    #  Type 1: AND direct   -- AND(EO-CP, AND(EC-OP, TPEnable))
    #    _spN_ctrl = AND(EC-OP, TPEnable)   [en gate]
    #    _spN_out  = AND(EO-CP, _spN_ctrl)  [main gate]
    #    Functional (TPEn=0): _ctrl=0 -> out=0  (forces 0 on EO-CP line)
    #    Test     (TPEn=1): _ctrl=EC-OP -> out=EO-CP & EC-OP
    #
    #  Type 2: AND inverted -- AND(EO-CP, NAND(EC-OP, TPEnable))
    #    _spN_ctrl = NAND(EC-OP, TPEnable)
    #    _spN_out  = AND(EO-CP, _spN_ctrl)
    #    Functional (TPEn=0): _ctrl=1 -> out=EO-CP (transparent)
    #    Test     (TPEn=1): _ctrl=~EC-OP -> inverted control
    #
    #  Type 3: OR direct    -- OR(EO-CP, AND(EC-OP, TPEnable))
    #    _spN_ctrl = AND(EC-OP, TPEnable)
    #    _spN_out  = OR(EO-CP, _spN_ctrl)
    #    Functional (TPEn=0): _ctrl=0 -> out=EO-CP (transparent)
    #    Test     (TPEn=1): _ctrl=EC-OP -> forces 1 when EC-OP=1
    #
    #  Type 4: OR inverted  -- OR(EO-CP, NOR(EC-OP, TPEnable))
    #    _spN_ctrl = NOR(EC-OP, TPEnable)
    #    _spN_out  = OR(EO-CP, _spN_ctrl)
    #    Functional (TPEn=0): _ctrl=~EC-OP -> inverted control
    #    Test     (TPEn=1): _ctrl=0 -> out=EO-CP (transparent)

    SP_EN_GATE  = {1: 'and',  2: 'nand', 3: 'and',  4: 'nor'}
    SP_MAIN_GATE= {1: 'and',  2: 'and',  3: 'or',   4: 'or'}

    for i, sp in enumerate(shared_points):
        ecp     = vlog_net(sp['eo_cp_net'])
        eop     = vlog_net(sp['ec_op_net'])
        sp_type = sp['sp_type']
        en_g    = SP_EN_GATE[sp_type]
        main_g  = SP_MAIN_GATE[sp_type]

        sp_lines.append(
            '  // SP{i}: Type {t} -- EO-CP={ecp}  EC-OP={eop}'
            '  CCreq={cr}  CCactual={ca}'.format(
                i=i, t=sp_type,
                ecp=sp['eo_cp_net'], eop=sp['ec_op_net'],
                cr=sp['cc_req'], ca=sp['cc_actual']))
        sp_lines.append(
            '  {eg} _sp{i}_en   (_sp{i}_ctrl, {eop}, TPEnable);'.format(
                eg=en_g, i=i, eop=eop))
        sp_lines.append(
            '  {mg} _sp{i}_gate (_sp{i}_out,  {ecp}, _sp{i}_ctrl);'.format(
                mg=main_g, i=i, ecp=ecp))
        sp_lines.append('')

    # Conventional CPs
    if conventional_cps:
        sp_lines.append('  // -- Conventional CPs (no EC-OP pair found) --')
        for net, cp_type in conventional_cps:
            gate = 'or' if cp_type == 'OR' else 'and'
            safe = re.sub(r'[^A-Za-z0-9_]', '_', net)
            vnet = vlog_net(net)
            sp_lines.append(
                '  {g} _conv_cp_{s} (_conv_cp_{s}_out, {n}, TPEnable);'
                '  // {t}-type CP'.format(
                    g=gate, s=safe, n=vnet, t=cp_type))
        sp_lines.append('')

    sp_block = '\n'.join(sp_lines) + '\n'

    # Insert before endmodule
    text = text.replace('endmodule', sp_block + 'endmodule', 1)

    # ----------------------------------------------------------
    # Write output
    # ----------------------------------------------------------
    out_dir = os.path.dirname(output_path)
    if out_dir and not os.path.exists(out_dir):
        os.makedirs(out_dir)

    with open(output_path, 'w') as f:
        f.write(text)

    print("[+] Modified netlist -> {0}".format(output_path))

    # Sanity check: count how many net references were renamed
    renamed_count = sum(
        text.count(new_net) for new_net in rename_map.values())
    print("    Net references renamed : {0} occurrences across {1} nets".format(
        renamed_count, len(rename_map)))


# ===============================================================
#  SECTION 12 - JSON REPORT
# ===============================================================

def write_json_report(shared_points, conventional_cps,
                      conventional_ops, output_path):
    out_dir = os.path.dirname(output_path)
    if out_dir and not os.path.exists(out_dir):
        os.makedirs(out_dir)
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
    print("[+] JSON report    -> {0}".format(output_path))


# ===============================================================
#  ENTRY POINT
# ===============================================================

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

    # Generate complete modified netlist (replaces patch generator)
    generate_full_netlist(sps, conv_cps, conv_ops, nl,
                          netlist_path,
                          "reports/spar_netlist.v")

    print("\nDone. Next step: read reports/spar_netlist.v into TetraMAX or DC.")