"""
find_reconverging_pair.py

# VERSION: 2026-06-22 rev1 -- initial version

Deliberately finds a real net inside an EO-CP's combinational-only
forward cone and tests Rule 1 / Rule 2 against it, since the official
EO-CP x EC-OP sample turned out to have zero combinational reconvergence
(confirmed via diagnose_reconvergence.py). This validates the rules'
mechanics on real gates/connectivity, independent of whether the
synthetic pairing happens to be an official TestMAX-flagged OP.

Run on synopsys01, same directory as COP.py / cone_analysis.py:

    python find_reconverging_pair.py netlist/ISCAS89/s38417.v \
        --tp_file netlist/TP_analysis/s38417_tp_analysis.txt
"""

from __future__ import print_function
import argparse
from collections import deque

import COP
import cone_analysis as CA


def forward_cone_with_path(start_net, graph, instances, stop_at_scan=True):
    """BFS forward from start_net, returning both the reachable set AND
    a parent-pointer map so we can reconstruct a sample path for
    sanity-checking by eye."""
    visited = {start_net: None}
    queue = deque([start_net])
    ff_bases = ('SDFFX1', 'SDFFX2', 'DFFX1')
    while queue:
        net = queue.popleft()
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
                    visited[onet] = net
                    queue.append(onet)
    return visited


def reconstruct_path(visited, target_net):
    path = [target_net]
    cur = target_net
    while visited.get(cur) is not None:
        cur = visited[cur]
        path.append(cur)
    path.reverse()
    return path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('netlist')
    parser.add_argument('--tp_file', required=True)
    args = parser.parse_args()

    print('Parsing:', args.netlist)
    ports_in, ports_out, instances, assigns, pin_to_net = \
        COP.parse_verilog(args.netlist)

    print('Running COP...')
    cc1_vals, co_vals = COP.run_cop(ports_in, ports_out, instances, assigns)

    print('Parsing TP file:', args.tp_file)
    cps, ops = COP.parse_tp_file(args.tp_file)

    eo_cps, ec_ops, CC_Th, CO_Th = COP.identify_candidates(
        cps, ops, cc1_vals, co_vals, pin_to_net, instances, verbose=False)

    sp_graph = CA.SharedPointGraph(instances)

    # Pick the first EO-CP and find a net partway through its
    # combinational cone to use as a synthetic "EC-OP"-like target.
    eo_ref, tp_type, obs = eo_cps[0]
    eo_net, _cc1 = COP.resolve_cp_net(eo_ref, tp_type, pin_to_net, cc1_vals, instances)
    print('\nUsing EO-CP: {} (net={})'.format(eo_ref, eo_net))

    visited = forward_cone_with_path(eo_net, sp_graph, instances, stop_at_scan=True)
    print('Combinational-only cone size: {}'.format(len(visited)))

    if len(visited) < 3:
        print('Cone too small to pick a meaningful midpoint net; '
            'try a different EO-CP from eo_cps list.')
        return

    # Pick a net that's a few hops downstream (not immediately adjacent,
    # not the EO-CP net itself) so the pairing isn't trivial.
    candidates = [n for n, parent in visited.items()
                if n != eo_net and parent is not None and parent != eo_net]
    if not candidates:
        candidates = [n for n in visited if n != eo_net]
    synthetic_ec_op_net = candidates[len(candidates) // 2]

    path = reconstruct_path(visited, synthetic_ec_op_net)
    print('Synthetic "EC-OP" net: {}'.format(synthetic_ec_op_net))
    print('Path from EO-CP to this net ({} hops): {}'.format(
        len(path) - 1, ' -> '.join(path)))

    # --- Now run the real Rule 1 / Rule 2 checks ---
    print('\n--- Rule 1 check ---')
    rule1_ok = CA.check_rule1(eo_net, synthetic_ec_op_net, sp_graph, instances)
    print('check_rule1(eo_net, synthetic_ec_op_net) = {}'.format(rule1_ok))
    print('Expected: False (UNSAFE) -- synthetic_ec_op_net is, by '
        'construction, in the EO-CP\'s forward cone, so pairing them '
        'would create exactly the Fig. 6 combinational loop scenario.')
    if rule1_ok:
        print('*** UNEXPECTED: Rule 1 says safe, but we built this pair '
            'specifically to be unsafe. This needs investigation. ***')
    else:
        print('Matches expectation: Rule 1 correctly detects the loop risk.')

    print('\n--- Rule 2 check (for context; Rule 1 already forbids this pair) ---')
    eo_required_value = 1 if tp_type == 'control_1' else 0
    blocked_cone_nets = {eo_net}
    rule2 = CA.check_rule2(eo_net, synthetic_ec_op_net, eo_required_value,
                            sp_graph, instances, cc1_vals, co_vals,
                            blocked_cone_nets)
    print('check_rule2 results: {}'.format(rule2))
    print('(Rule 2 evaluating a pair Rule 1 already forbids is just for '
        'visibility into Rule 2\'s mechanics here, not a real candidate)')


if __name__ == '__main__':
    main()
