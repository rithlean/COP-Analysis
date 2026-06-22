"""
run_cone_analysis.py

Driver script: wires COP.py's EO-CP/EC-OP candidate identification
into cone_analysis.py's Rule 1 / Rule 2 checks, on a real netlist.

Run on synopsys01, in the same directory as COP.py and cone_analysis.py:

    python run_cone_analysis.py netlist/ISCAS89/s38417.v \
        --tp_file netlist/TP_analysis/s38417_tp_analysis.txt

KNOWN SIMPLIFICATION (flagged, not hidden): Rule 2's blocked_cone_nets
argument is approximated here as {eo_cp_net} itself -- i.e. we check
whether pairing with a given EC-OP would drive the EO-CP line's own
observability to zero THROUGH THIS SPECIFIC PAIRING'S new local
wiring. This only has teeth when the EC-OP's forward propagation
actually reconverges with the EO-CP net (matching Fig. 8's setup) --
for the common case of two structurally unrelated nets in a large
netlist, there is no reconvergence, so Rule 2 correctly reports safe
(not because the check is vacuous, but because there is genuinely no
interaction to flag). The paper's Fig. 9 example traces a SPECIFIC
upstream fanin cone (ConeG6) feeding INTO the gate where the EO-CP and
EC-OP paths reconverge; using {eo_cp_net} as a stand-in for that cone's
output is a reasonable approximation for a first real-data test, but
the fully faithful version would identify the EO-CP's actual fanin
cone gates specifically rather than just the EO-CP net.
"""

from __future__ import print_function
import argparse

import COP
import cone_analysis as CA


def resolve_eo_cp(ref, tp_type, pin_to_net, cc1_vals, instances):
    """Resolve an EO-CP TetraMAX ref to its controlled net, reusing
    COP.py's own inconsistency-aware resolution."""
    net, cc1 = COP.resolve_cp_net(ref, tp_type, pin_to_net, cc1_vals, instances)
    return net


def resolve_ec_op(ref, pin_to_net, cc1_vals):
    """Resolve an EC-OP TetraMAX ref to its net via plain lookup."""
    lookup = COP.make_lookup(pin_to_net, cc1_vals)
    return lookup(ref)


def main():
    parser = argparse.ArgumentParser(
        description='Cone analysis (Rule 1 / Rule 2) smoke test on real netlist')
    parser.add_argument('netlist')
    parser.add_argument('--tp_file', required=True)
    parser.add_argument('--max_pairs', type=int, default=300,
                        help='Cap on number of EO-CP x EC-OP pairs to check '
                            '(can be O(EO-CPs * EC-OPs), keep bounded for a smoke test). '
                            'Budget is distributed evenly across EO-CPs.')
    args = parser.parse_args()

    print('Parsing:', args.netlist)
    ports_in, ports_out, instances, assigns, pin_to_net = \
        COP.parse_verilog(args.netlist)

    print('Running COP...')
    cc1_vals, co_vals = COP.run_cop(ports_in, ports_out, instances, assigns)

    print('Parsing TP file:', args.tp_file)
    cps, ops = COP.parse_tp_file(args.tp_file)

    eo_cps, ec_ops, CC_Th, CO_Th = COP.identify_candidates(
        cps, ops, cc1_vals, co_vals, pin_to_net, instances, verbose=True)

    print('\nBuilding fanout graph for cone analysis...')
    graph = CA.SharedPointGraph(instances)

    lookup = COP.make_lookup(pin_to_net, cc1_vals)

    print('\n--- Cone analysis: checking EO-CP x EC-OP pairs ---')
    print('(blocked_cone_nets = {eo_cp_net}; Rule 2 only triggers on real reconvergence -- see module docstring)\n')

    n_checked = 0
    n_rule1_pass = 0
    n_rule2_direct_pass = 0
    n_rule2_inverted_pass = 0
    n_both_rules_pass = 0
    errors = []

    n_eo_cps_with_net = sum(
        1 for eo_ref, tp_type, obs in eo_cps
        if resolve_eo_cp(eo_ref, tp_type, pin_to_net, cc1_vals, instances) is not None)
    per_eo_cp_budget = max(1, args.max_pairs // max(1, n_eo_cps_with_net))
    print('  ({} EO-CPs with resolvable nets; budget ~{} EC-OPs checked per EO-CP)'.format(
        n_eo_cps_with_net, per_eo_cp_budget))

    for eo_ref, tp_type, obs in eo_cps:
        eo_net = resolve_eo_cp(eo_ref, tp_type, pin_to_net, cc1_vals, instances)
        if eo_net is None:
            continue

        # OR-type CP (control_1) needs the line driven to 1 to propagate
        # faults; AND-type CP (control_0) needs it driven to 0.
        # (Section III-A: hard-to-control-1 -> OR-type CP -> inject 1.)
        eo_required_value = 1 if tp_type == 'control_1' else 0

        n_for_this_eo_cp = 0
        for ec_ref, ctrl in ec_ops:
            if n_checked >= args.max_pairs:
                break
            if n_for_this_eo_cp >= per_eo_cp_budget:
                break
            ec_net = resolve_ec_op(ec_ref, pin_to_net, cc1_vals)
            if ec_net is None or ec_net == eo_net:
                continue

            n_checked += 1
            n_for_this_eo_cp += 1
            try:
                rule1_ok = CA.check_rule1(eo_net, ec_net, graph, instances)
            except Exception as e:
                errors.append(('rule1', eo_ref, ec_ref, repr(e)))
                continue

            if rule1_ok:
                n_rule1_pass += 1

            try:
                blocked_cone_nets = {eo_net}
                rule2 = CA.check_rule2(
                    eo_net, ec_net, eo_required_value, graph, instances,
                    cc1_vals, co_vals, blocked_cone_nets)
            except Exception as e:
                errors.append(('rule2', eo_ref, ec_ref, repr(e)))
                continue

            if rule2['direct']:
                n_rule2_direct_pass += 1
            if rule2['inverted']:
                n_rule2_inverted_pass += 1
            if rule1_ok and (rule2['direct'] or rule2['inverted']):
                n_both_rules_pass += 1
                print('  PAIR OK   EO-CP={:30s} EC-OP={:30s} '
                    'rule1={} rule2_direct={} rule2_inverted={}'.format(
                        eo_ref, ec_ref, rule1_ok,
                        rule2['direct'], rule2['inverted']))

        if n_checked >= args.max_pairs:
            break

    print('\n--- Summary ---')
    print('  Pairs checked:        {}'.format(n_checked))
    print('  Rule 1 pass:          {}'.format(n_rule1_pass))
    print('  Rule 2 direct pass:   {}'.format(n_rule2_direct_pass))
    print('  Rule 2 inverted pass: {}'.format(n_rule2_inverted_pass))
    print('  Both rules pass (viable pairs): {}'.format(n_both_rules_pass))

    if errors:
        print('\n--- Errors encountered ({}) ---'.format(len(errors)))
        for kind, eo_ref, ec_ref, msg in errors[:20]:
            print('  [{}] EO-CP={} EC-OP={}: {}'.format(kind, eo_ref, ec_ref, msg))
        if len(errors) > 20:
            print('  ... and {} more'.format(len(errors) - 20))


if __name__ == '__main__':
    main()
