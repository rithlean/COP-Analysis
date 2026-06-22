"""
diagnose_reconvergence.py

Sanity check for the 288/288 pass rate from run_cone_analysis.py.
Distinguishes two possibilities:
  (a) Genuine: these EO-CP/EC-OP pairs really don't share any forward
      path or convergence point in this netlist's topology.
  (b) Bug: the fanout graph isn't capturing real connectivity for
      these specific nets (e.g. a net-name mismatch between
      resolve_cp_net's output and what's actually in instances'
      conn dicts).

Run on synopsys01, same directory as COP.py / cone_analysis.py:

    python diagnose_reconvergence.py netlist/ISCAS89/s38417.v \
        --tp_file netlist/TP_analysis/s38417_tp_analysis.txt
"""

from __future__ import print_function
import argparse
from collections import deque

import COP
import cone_analysis as CA


def count_forward_reachable(start_net, graph, instances, max_visit=None,
                            stop_at_scan=True):
    """BFS forward from start_net, return the set of all reachable nets
    (no target, just full reachability size) -- a sanity check on
    whether the graph is connected the way a 5133-instance netlist
    should be.

    stop_at_scan mirrors cone_analysis.is_reachable's actual behavior:
    Rule 1 only cares about COMBINATIONAL loops, so a path that only
    exists by crossing through a flip-flop's D->Q is not a loop risk
    and must NOT count as reachable here, or this diagnostic measures
    a different (larger) graph than what check_rule1 actually checks.
    """
    visited = set([start_net])
    queue = deque([start_net])
    ff_bases = ('SDFFX1', 'SDFFX2', 'DFFX1')
    while queue:
        net = queue.popleft()
        if max_visit and len(visited) >= max_visit:
            break
        for (inst_idx, _ip) in graph.net_to_sinks.get(net, []):
            inst = instances[inst_idx]
            base = COP.get_cell_base(inst['cell'])
            if stop_at_scan and base in ff_bases:
                continue
            info = COP.CELL_OUTPUT_PORTS.get(base)
            if info is None:
                continue
            _in_ports, out_ports = info
            for op in out_ports:
                onet = inst['conn'].get(op)
                if onet and onet not in visited:
                    visited.add(onet)
                    queue.append(onet)
    return visited


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('netlist')
    parser.add_argument('--tp_file', required=True)
    args = parser.parse_args()

    print('Parsing:', args.netlist)
    ports_in, ports_out, instances, assigns, pin_to_net = \
        COP.parse_verilog(args.netlist)
    print('  instances={}'.format(len(instances)))

    print('Running COP...')
    cc1_vals, co_vals = COP.run_cop(ports_in, ports_out, instances, assigns)

    print('Parsing TP file:', args.tp_file)
    cps, ops = COP.parse_tp_file(args.tp_file)

    eo_cps, ec_ops, CC_Th, CO_Th = COP.identify_candidates(
        cps, ops, cc1_vals, co_vals, pin_to_net, instances, verbose=False)

    sp_graph = CA.SharedPointGraph(instances)

    lookup = COP.make_lookup(pin_to_net, cc1_vals)

    # --- Check 1: graph connectivity sanity, independent of any pair ---
    print('\n--- Check 1: forward reachability size from each EO-CP net ---')
    print('(if these are tiny/zero on a 5133-instance netlist, the graph')
    print(' construction itself is suspect, not the no-reconvergence claim)\n')

    eo_nets = []
    for eo_ref, tp_type, obs in eo_cps:
        eo_net, _cc1 = COP.resolve_cp_net(eo_ref, tp_type, pin_to_net, cc1_vals, instances)
        if eo_net is None:
            print('  {:30s} -> COULD NOT RESOLVE NET'.format(eo_ref))
            continue
        eo_nets.append((eo_ref, eo_net))
        reachable_comb = count_forward_reachable(
            eo_net, sp_graph, instances, max_visit=2000, stop_at_scan=True)
        reachable_all = count_forward_reachable(
            eo_net, sp_graph, instances, max_visit=2000, stop_at_scan=False)
        print('  {:30s} net={:15s} combinational-only={:5d}  through-FFs={:5d}'.format(
            eo_ref, eo_net, len(reachable_comb), len(reachable_all)))

    # --- Check 2: are EC-OP nets actually IN any EO-CP's reachable set? ---
    print('\n--- Check 2: do EC-OP nets fall inside ANY EO-CP forward cone? ---\n')

    ec_nets = []
    for ec_ref, ctrl in ec_ops:
        ec_net = lookup(ec_ref)
        if ec_net:
            ec_nets.append((ec_ref, ec_net))

    if eo_nets:
        first_eo_ref, first_eo_net = eo_nets[0]
        full_reachable_comb = count_forward_reachable(
            first_eo_net, sp_graph, instances, stop_at_scan=True)
        full_reachable_all = count_forward_reachable(
            first_eo_net, sp_graph, instances, stop_at_scan=False)
        print('  Combinational-only reachability from {} ({}): {} nets'.format(
            first_eo_ref, first_eo_net, len(full_reachable_comb)))
        print('  Through-FFs reachability from {} ({}): {} nets'.format(
            first_eo_ref, first_eo_net, len(full_reachable_all)))
        hits_comb = [ec_ref for ec_ref, ec_net in ec_nets if ec_net in full_reachable_comb]
        hits_all = [ec_ref for ec_ref, ec_net in ec_nets if ec_net in full_reachable_all]
        print('  Of {} sampled EC-OP nets: {} fall inside COMBINATIONAL-ONLY cone, '
            '{} fall inside THROUGH-FFS cone'.format(
                len(ec_nets), len(hits_comb), len(hits_all)))
        print('  Combinational-only hits (these are the ones Rule1 should reject):', hits_comb[:10])
        print('  (the difference between these two counts is reconvergence that ONLY')
        print('   exists by crossing a scan flip-flop, which Rule 1 correctly ignores)')

    # --- Check 3: do the resolved nets actually appear as keys in
    # net_to_sinks or net_to_driver_port at all? (sanity check that
    # resolve_cp_net's output net names match what's really in the
    # instances' conn dicts) ---
    print('\n--- Check 3: do resolved EO-CP/EC-OP nets appear in the graph at all? ---\n')
    for eo_ref, eo_net in eo_nets[:5]:
        in_sinks = eo_net in sp_graph.net_to_sinks
        has_driver = eo_net in sp_graph.net_to_driver_port
        print('  EO-CP {:20s} net={:15s} has_sinks={} has_driver={}'.format(
            eo_ref, eo_net, in_sinks, has_driver))
    for ec_ref, ec_net in ec_nets[:5]:
        in_sinks = ec_net in sp_graph.net_to_sinks
        has_driver = ec_net in sp_graph.net_to_driver_port
        print('  EC-OP {:20s} net={:15s} has_sinks={} has_driver={}'.format(
            ec_ref, ec_net, in_sinks, has_driver))


if __name__ == '__main__':
    main()
