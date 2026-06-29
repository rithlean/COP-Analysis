"""
test_bB_fig6.py

# VERSION: 2026-06-25 rev1 -- initial version

Tests compute_b_and_B against [28]'s Fig. 6 (Moghaddam et al.,
"Logic BIST With Capture-Per-Clock Hybrid Test Points"), gate shapes
confirmed via image zoom:

  G3 = NAND2(x, y) -> z0
  z0 fans out to z1, z2 (no gates in between, per the figure)
  G4 = AND2(z1, D1_stem) -> feeds G6
  G6 = OR2(G4_out, D3_stem) -> output
  G5 = OR2(z2, D2_stem) -> feeds G7 and G8
  G7 = AND2(G5_out, D4_stem) -> output
  G8 = AND2(G5_out, D5_stem) -> output

Expected (from the figure's own given labels):
  b_z1 = 0          B_z1 = D1
  b_z2 = D2          B_z2 = 0
  b_z0 = b_z1+b_z2 = D2     B_z0 = B_z1+B_z2 = D1

We use D1=D2=D3=D4=D5=1 fault each (the simplest seed) to make the
arithmetic checkable by eye, then verify the RATIOS/relationships
match the figure's structure exactly.
"""
import sys
sys.path.insert(0, '.')
import types

COP = types.ModuleType('COP')
COP.CELL_OUTPUT_PORTS = {
    'NAND2': (['A1', 'A2'], ['Y']),
    'AND2':  (['A1', 'A2'], ['Y']),
    'OR2':   (['A1', 'A2'], ['Y']),
}
COP.get_cell_base = lambda c: c
sys.modules['COP'] = COP

import cone_analysis as CA

instances = [
    {'cell': 'NAND2', 'inst': 'G3', 'conn': {'A1': 'x', 'A2': 'y', 'Y': 'z0'}},
    {'cell': 'AND2',  'inst': 'G4', 'conn': {'A1': 'z1', 'A2': 'D1_stem', 'Y': 'g4out'}},
    {'cell': 'OR2',   'inst': 'G6', 'conn': {'A1': 'g4out', 'A2': 'D3_stem', 'Y': 'g6out'}},
    {'cell': 'OR2',   'inst': 'G5', 'conn': {'A1': 'z2', 'A2': 'D2_stem', 'Y': 'g5out'}},
    {'cell': 'AND2',  'inst': 'G7', 'conn': {'A1': 'g5out', 'A2': 'D4_stem', 'Y': 'g7out'}},
    {'cell': 'AND2',  'inst': 'G8', 'conn': {'A1': 'g5out', 'A2': 'D5_stem', 'Y': 'g8out'}},
]

# z0 fans out to z1 and z2 directly (same net value, no gate in
# between per the figure) -- model this with a synthetic identity:
# since our framework needs z1/z2 to be DISTINCT net names that G4/G5
# read, but z0 has no explicit fanout gates in the figure, we add
# trivial BUF passthroughs to give z1, z2 their own net identities
# while preserving "same value as z0".
COP.CELL_OUTPUT_PORTS['BUF'] = (['A'], ['Y'])
instances = [
    {'cell': 'NAND2', 'inst': 'G3',  'conn': {'A1': 'x', 'A2': 'y', 'Y': 'z0'}},
    {'cell': 'BUF',   'inst': 'B1',  'conn': {'A': 'z0', 'Y': 'z1'}},
    {'cell': 'BUF',   'inst': 'B2',  'conn': {'A': 'z0', 'Y': 'z2'}},
    {'cell': 'AND2',  'inst': 'G4',  'conn': {'A1': 'z1', 'A2': 'D1_stem', 'Y': 'g4out'}},
    {'cell': 'OR2',   'inst': 'G6',  'conn': {'A1': 'g4out', 'A2': 'D3_stem', 'Y': 'g6out'}},
    {'cell': 'OR2',   'inst': 'G5',  'conn': {'A1': 'z2', 'A2': 'D2_stem', 'Y': 'g5out'}},
    {'cell': 'AND2',  'inst': 'G7',  'conn': {'A1': 'g5out', 'A2': 'D4_stem', 'Y': 'g7out'}},
    {'cell': 'AND2',  'inst': 'G8',  'conn': {'A1': 'g5out', 'A2': 'D5_stem', 'Y': 'g8out'}},
]

graph = CA.SharedPointGraph(instances)

# D_i = 1 for each stem (simplest seed, makes ratios checkable by eye)
fault_counts = {
    'D1_stem': 1.0, 'D2_stem': 1.0, 'D3_stem': 1.0,
    'D4_stem': 1.0, 'D5_stem': 1.0,
}

print("=== b_z1, B_z1 ===")
b_z1, B_z1 = CA.compute_b_and_B('z1', graph, instances, fault_counts)
print('b_z1={}, B_z1={}'.format(b_z1, B_z1))
print('Expected: b_z1=0 (z1=1 is NON-dominant for AND2, nothing blocked)')
print('          B_z1=D1=1.0 (z1=0 IS dominant for AND2, blocks D1_stem)')
assert b_z1 == 0.0, "FAIL: b_z1 should be 0"
assert B_z1 == 1.0, "FAIL: B_z1 should be D1=1.0, got {}".format(B_z1)
print("PASS\n")

print("=== b_z2, B_z2 ===")
b_z2, B_z2 = CA.compute_b_and_B('z2', graph, instances, fault_counts)
print('b_z2={}, B_z2={}'.format(b_z2, B_z2))
print('Expected: b_z2=D2=1.0 (z2=1 IS dominant for OR2, blocks D2_stem)')
print('          B_z2=0 (z2=0 is NON-dominant for OR2, nothing blocked)')
assert b_z2 == 1.0, "FAIL: b_z2 should be D2=1.0, got {}".format(b_z2)
assert B_z2 == 0.0, "FAIL: B_z2 should be 0, got {}".format(B_z2)
print("PASS\n")

print("=== b_z0, B_z0 (should equal sum of branches per Eq.2/3) ===")
b_z0, B_z0 = CA.compute_b_and_B('z0', graph, instances, fault_counts)
print('b_z0={}, B_z0={}'.format(b_z0, B_z0))
print('Expected: b_z0 = b_z1+b_z2 = 0+1 = 1.0')
print('          B_z0 = B_z1+B_z2 = 1+0 = 1.0')
assert b_z0 == 1.0, "FAIL: b_z0 should be 1.0, got {}".format(b_z0)
assert B_z0 == 1.0, "FAIL: B_z0 should be 1.0, got {}".format(B_z0)
print("PASS\n")

print("ALL FIG.6 b/B TESTS PASSED -- matches the paper's own worked example")
