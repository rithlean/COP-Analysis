"""
test_fault_counts.py

# VERSION: 2026-06-25 rev1 -- initial version

Tests compute_fault_counts (Eq.1 forward fault-count propagation)
against a small hand-traceable circuit:

  pi1, pi2 -> U1(AND2) -> mid     (D_mid should be D_pi1 + D_pi2 + 1
                                    = 1 + 1 + 1 = 3, since each net
                                    seeds 1 and AND2 sums its inputs)
  mid -> U2(BUF) -> out           (D_out = D_mid + 1 = 4)

This confirms the additive accumulation rule (sum of inputs + own
seed) works correctly before testing fanout-split behavior separately.
"""
import sys
sys.path.insert(0, '.')
import types

COP = types.ModuleType('COP')
COP.CELL_OUTPUT_PORTS = {
    'AND2': (['A1', 'A2'], ['Y']),
    'BUF':  (['A'], ['Y']),
}
COP.get_cell_base = lambda c: c
sys.modules['COP'] = COP

import cone_analysis as CA

instances = [
    {'cell': 'AND2', 'inst': 'U1', 'conn': {'A1': 'pi1', 'A2': 'pi2', 'Y': 'mid'}},
    {'cell': 'BUF',  'inst': 'U2', 'conn': {'A': 'mid', 'Y': 'out'}},
]
ports_in = {'pi1', 'pi2'}
topo = [0, 1]  # U1 before U2, matching the dependency order

D = CA.compute_fault_counts(ports_in, instances, [], {}, {}, topo, None)

print('D:', D)
assert D['pi1'] == 1.0, "FAIL: PI seed should be 1.0"
assert D['pi2'] == 1.0, "FAIL: PI seed should be 1.0"
assert D['mid'] == 3.0, "FAIL: mid should be pi1(1) + pi2(1) + own seed(1) = 3.0, got {}".format(D['mid'])
assert D['out'] == 4.0, "FAIL: out should be mid(3) + own seed(1) = 4.0, got {}".format(D['out'])
print("PASS -- additive accumulation confirmed correct")
