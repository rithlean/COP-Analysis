"""
check_g23359.py

# VERSION: 2026-06-23 rev1 -- initial version

Standalone check: is g23359 inside n3384's (U5181/Y's) combinational-only
forward cone? Self-contained -- doesn't require importing two other
scripts together.

Run on synopsys01, same directory as COP.py / cone_analysis.py:

    python check_g23359.py netlist/ISCAS89/s38417.v
"""

from __future__ import print_function
import argparse
from collections import deque

import COP
import cone_analysis as CA


def forward_cone(start_net, sp_graph, instances, stop_at_scan=True):
    visited = {start_net: None}
    queue = deque([start_net])
    ff_bases = ('SDFFX1', 'SDFFX2', 'DFFX1')
    while queue:
        net = queue.popleft()
        for (inst_idx, _ip) in sp_graph.net_to_sinks.get(net, []):
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
    if target_net not in visited:
        return None
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
    args = parser.parse_args()

    print('Parsing:', args.netlist)
    ports_in, ports_out, instances, assigns, pin_to_net = \
        COP.parse_verilog(args.netlist)

    sp_graph = CA.SharedPointGraph(instances)

    eo_net = 'n3384'
    target_net = 'g23359'

    print('Computing combinational-only forward cone from {}...'.format(eo_net))
    visited_comb = forward_cone(eo_net, sp_graph, instances, stop_at_scan=True)
    print('Cone size (combinational-only): {}'.format(len(visited_comb)))

    in_comb_cone = target_net in visited_comb
    print('\n{} in combinational-only forward cone of {}? {}'.format(
        target_net, eo_net, in_comb_cone))
    if in_comb_cone:
        path = reconstruct_path(visited_comb, target_net)
        print('Path: {}'.format(' -> '.join(path)))

    print('\nComputing through-FFs forward cone from {} (for context)...'.format(eo_net))
    visited_all = forward_cone(eo_net, sp_graph, instances, stop_at_scan=False)
    print('Cone size (through-FFs): {}'.format(len(visited_all)))
    in_all_cone = target_net in visited_all
    print('{} in through-FFs forward cone of {}? {}'.format(
        target_net, eo_net, in_all_cone))
    if in_all_cone and not in_comb_cone:
        path = reconstruct_path(visited_all, target_net)
        print('Path (crosses at least one FF): {}'.format(' -> '.join(path)))

    # Also check the reverse direction: is eo_net reachable FORWARD from
    # target_net? (i.e. is target_net actually UPSTREAM of eo_net, which
    # would explain check_rule1=False differently)
    print('\nChecking reverse direction: is {} reachable forward from {}?'.format(
        eo_net, target_net))
    visited_from_target = forward_cone(target_net, sp_graph, instances, stop_at_scan=True)
    eo_reachable_from_target = eo_net in visited_from_target
    print('{} reachable forward from {} (combinational-only)? {}'.format(
        eo_net, target_net, eo_reachable_from_target))
    if eo_reachable_from_target:
        path = reconstruct_path(visited_from_target, eo_net)
        print('Path: {}'.format(' -> '.join(path)))


if __name__ == '__main__':
    main()
