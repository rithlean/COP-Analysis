"""
find_drivers.py

# VERSION: 2026-06-23 rev1 -- initial version

Given a netlist and a list of net names, finds which instance drives
each net (i.e. has it as an output) and prints that instance's full
cell type + connections -- so we can trace backward from a net to its
source, the missing piece needed to hand-verify the s9234 Rule 2 trace.

Run on synopsys01:

    python find_drivers.py netlist/ISCAS89/s9234.v n375 g687 n369
"""

from __future__ import print_function
import argparse

import COP


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('netlist')
    parser.add_argument('nets', nargs='+')
    args = parser.parse_args()

    print('Parsing:', args.netlist)
    ports_in, ports_out, instances, assigns, pin_to_net = \
        COP.parse_verilog(args.netlist)

    assign_map = {lhs: rhs for lhs, rhs in assigns}

    for net in args.nets:
        print('\n=== {} ==='.format(net))
        if net in ports_in:
            print('  This is a PRIMARY INPUT -- no driver gate.')
            continue
        if net in assign_map:
            print('  assign alias: {} = {}'.format(net, assign_map[net]))

        found_driver = False
        for inst in instances:
            base = COP.get_cell_base(inst['cell'])
            info = COP.CELL_OUTPUT_PORTS.get(base)
            if info is None:
                continue
            _in_ports, out_ports = info
            for op in out_ports:
                if inst['conn'].get(op) == net:
                    print('  DRIVEN BY: {} ({})'.format(inst['inst'], inst['cell']))
                    print('    conn = {}'.format(inst['conn']))
                    found_driver = True
        if not found_driver:
            print('  No driver instance found (may be a PI, or unresolved).')

        # Also show what CONSUMES this net (its sinks), for context
        print('  CONSUMED BY:')
        for inst in instances:
            base = COP.get_cell_base(inst['cell'])
            info = COP.CELL_OUTPUT_PORTS.get(base)
            if info is None:
                continue
            in_ports, _out_ports = info
            for ip in in_ports:
                if inst['conn'].get(ip) == net:
                    print('    {} ({}) as port {}'.format(inst['inst'], inst['cell'], ip))


if __name__ == '__main__':
    main()
