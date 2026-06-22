"""
find_rule2_convergence.py

# VERSION: 2026-06-22 rev1 -- initial version

Finds a REAL Fig.8-style reconvergent pair to properly exercise Rule 2,
correcting find_reconverging_pair.py's topology mismatch: that script
picked a net simply downstream of the EO-CP net, which tests Rule 1's
loop detection but gives Rule 2 nothing meaningful to evaluate (forward
propagation from a downstream net never walks back to a gate that has
the EO-CP net as a direct input).

Rule 2 needs: a gate G where eo_net is one direct input, and a
DIFFERENT net (call it candidate_ec_net) that feeds G's OTHER input
through some local logic. Forward-propagating from candidate_ec_net
will then reach G -- the actual convergence point -- giving Rule 2 a
real gate to evaluate, matching Fig. 8/9's structure.

Run on synopsys01, same directory as COP.py / cone_analysis.py:

    python find_rule2_convergence.py netlist/ISCAS89/s38417.v \
        --tp_file netlist/TP_analysis/s38417_tp_analysis.txt
"""

from __future__ import print_function
import argparse
from collections import deque

import COP
import cone_analysis as CA


def find_convergence_gate(eo_net, sp_graph, instances):
    """Find a gate where eo_net is a direct input, and return
    (gate_idx, other_input_net) for the first such gate found whose
    other input is a distinct, resolvable net."""
    for (inst_idx, ip) in sp_graph.net_to_sinks.get(eo_net, []):
        inst = instances[inst_idx]
        base = COP.get_cell_base(inst['cell'])
        info = COP.CELL_OUTPUT_PORTS.get(base)
        if info is None:
            continue
        in_ports, _out_ports = info
        conn = inst['conn']
        for other_port in in_ports:
            if other_port == ip:
                continue
            other_net = conn.get(other_port)
            if other_net and other_net != eo_net:
                return inst_idx, other_net
    return None, None


def backward_sample(net, sp_graph, instances, hops=2):
    """Walk backward a few hops from `net` via its driver, to find a
    net further upstream to use as a synthetic EC-OP -- so that
    propagating FORWARD from it will naturally flow into `net` and
    onward to the convergence gate, rather than starting exactly at
    the convergence gate's doorstep (which would be a trivial,
    uninteresting test)."""
    cur = net
    for _ in range(hops):
        drv = sp_graph.net_to_driver_port.get(cur)
        if drv is None:
            break
        inst_idx, _port = drv
        inst = instances[inst_idx]
        base = COP.get_cell_base(inst['cell'])
        info = COP.CELL_OUTPUT_PORTS.get(base)
        if info is None:
            break
        in_ports, _out_ports = info
        conn = inst['conn']
        upstream_nets = [conn.get(p) for p in in_ports if conn.get(p)]
        if not upstream_nets:
            break
        cur = upstream_nets[0]
    return cur


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

    found = False
    for eo_ref, tp_type, obs in eo_cps:
        eo_net, _cc1 = COP.resolve_cp_net(eo_ref, tp_type, pin_to_net, cc1_vals, instances)
        if eo_net is None:
            continue

        conv_idx, other_net = find_convergence_gate(eo_net, sp_graph, instances)
        if conv_idx is None:
            print('EO-CP {} (net={}): no usable convergence gate found, trying next EO-CP'.format(
                eo_ref, eo_net))
            continue

        conv_inst = instances[conv_idx]
        print('\nUsing EO-CP: {} (net={})'.format(eo_ref, eo_net))
        print('Convergence gate: {} ({}), other input net: {}'.format(
            conv_inst['inst'], conv_inst['cell'], other_net))

        synthetic_ec_op_net = backward_sample(other_net, sp_graph, instances, hops=2)
        print('Synthetic "EC-OP" net (backward-sampled): {}'.format(synthetic_ec_op_net))

        # --- Rule 1: this pair should likely be SAFE (ec_net feeds the
        # SAME downstream gate as eo_net, but isn't IN eo_net's own
        # forward cone -- reconvergence, not a simple loop) ---
        print('\n--- Rule 1 check ---')
        rule1_ok = CA.check_rule1(eo_net, synthetic_ec_op_net, sp_graph, instances)
        print('check_rule1 = {}'.format(rule1_ok))

        # --- Rule 2: THIS is the real test. Forward injection from
        # synthetic_ec_op_net should reach conv_idx (since that's
        # literally how we constructed it), giving Rule 2 a genuine
        # convergence point with eo_net as the blocked-cone net. ---
        print('\n--- Rule 2 check (the actual test) ---')
        eo_required_value = 1 if tp_type == 'control_1' else 0
        blocked_cone_nets = {eo_net}
        rule2 = CA.check_rule2(eo_net, synthetic_ec_op_net, eo_required_value,
                                sp_graph, instances, cc1_vals, co_vals,
                                blocked_cone_nets)
        print('check_rule2 results: {}'.format(rule2))

        # Diagnostic: confirm the forward injection actually DID reach
        # the convergence gate, so we know whether the test setup
        # worked or whether we need a different EO-CP.
        stop_nets = {eo_net, synthetic_ec_op_net}
        for inj_val in (0, 1):
            forced_cc, last_gates = CA.propagate_forward_injection(
                synthetic_ec_op_net, inj_val, sp_graph, instances,
                stop_nets, convergence_nets=blocked_cone_nets)
            reached_conv = conv_idx in last_gates
            print('  inject_val={}: last_gates includes convergence gate '
                '{}? {}  (last_gates={})'.format(
                    inj_val, conv_inst['inst'], reached_conv,
                    [instances[i]['inst'] for i in last_gates][:10]))

        found = True
        break

    if not found:
        print('\nNo usable EO-CP/convergence-gate pair found across all EO-CPs.')


if __name__ == '__main__':
    main()
