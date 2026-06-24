"""
inspect_gates.py

# VERSION: 2026-06-23 rev1 -- initial version

Prints the cell type and full connection dict for specific instances,
so we can hand-verify a Rule 2 trace against the REAL gate logic
instead of assuming the trace is correct just because it ran.

Run on synopsys01, same directory as COP.py:

    python inspect_gates.py netlist/ISCAS89/s9234.v U525 U535 U537 U543
"""

from __future__ import print_function
import argparse

import COP


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('netlist')
    parser.add_argument('inst_names', nargs='+',
                        help='Instance names to inspect, e.g. U525 U535')
    args = parser.parse_args()

    print('Parsing:', args.netlist)
    ports_in, ports_out, instances, assigns, pin_to_net = \
        COP.parse_verilog(args.netlist)

    by_name = {inst['inst']: inst for inst in instances}
    # Also try with leading backslash variants, since some instance
    # names in real netlists carry escape characters.
    for inst in instances:
        alt = inst['inst'].lstrip('\\')
        by_name.setdefault(alt, inst)

    for name in args.inst_names:
        inst = by_name.get(name)
        if inst is None:
            print('\n{}: NOT FOUND'.format(name))
            continue
        base = COP.get_cell_base(inst['cell'])
        info = COP.CELL_OUTPUT_PORTS.get(base)
        print('\n{}:'.format(name))
        print('  cell      = {}'.format(inst['cell']))
        print('  base      = {}'.format(base))
        print('  in CELL_OUTPUT_PORTS? {}'.format(info is not None))
        if info:
            print('  in_ports  = {}'.format(info[0]))
            print('  out_ports = {}'.format(info[1]))
        print('  conn      = {}'.format(inst['conn']))


if __name__ == '__main__':
    main()
