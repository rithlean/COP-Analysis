"""
cone_analysis.py

SPAR Section V: Cone Analysis.
Implements Rule 1 (combinational loop generation) and Rule 2
(fault-propagation block) checks for candidate EO-CP / EC-OP pairs,
on top of the COP engine in COP.py.

Based on: Kim, Cheong, Kang, "SPAR: A New Test-Point Insertion Using
Shared Points for Area Overhead Reduction," IEEE TCAD, vol. 41, no. 11,
Nov 2022, Section V.

Usage:
    import COP
    import cone_analysis as CA

    ports_in, ports_out, instances, assigns, pin_to_net = \
        COP.parse_verilog(netlist_path)
    cc1_vals, co_vals = COP.run_cop(ports_in, ports_out, instances, assigns)

    graph = CA.build_fanout_graph(instances)
    ok = CA.check_rule1(eo_cp_net, ec_op_net, graph)
"""

from collections import deque

import COP


# ---------------------------------------------------------------------------
# FORWARD FANOUT GRAPH  (net -> consumers)
# ---------------------------------------------------------------------------

def build_fanout_graph(instances):
    """
    Build a net -> [(inst_idx, input_port), ...] map by scanning every
    instance's connection dict directly. This is independent of COP.py's
    internal topo-sort machinery (gate_fanout there is gate->gate, local
    to build_and_sort, and not exposed).

    Independent of build_and_sort by design: cone analysis needs to keep
    adding synthetic edges as shared points are committed during a SPAR
    run, without re-deriving a topological order each time.

    Returns:
        net_to_sinks : dict  net -> list of (inst_idx, input_port)
        net_to_driver_port : dict  net -> (inst_idx, output_port)
    """
    net_to_sinks = {}
    net_to_driver_port = {}

    for idx, inst in enumerate(instances):
        base = COP.get_cell_base(inst['cell'])
        info = COP.CELL_OUTPUT_PORTS.get(base)
        if info is None:
            continue
        in_ports, out_ports = info
        conn = inst['conn']

        for ip in in_ports:
            net = conn.get(ip)
            if net:
                net_to_sinks.setdefault(net, []).append((idx, ip))

        for op in out_ports:
            net = conn.get(op)
            if net:
                net_to_driver_port[net] = (idx, op)

    return net_to_sinks, net_to_driver_port


class SharedPointGraph(object):
    """
    Wraps the static fanout graph plus a growing set of synthetic
    EC-OP -> EO-CP edges representing shared points already committed
    in this SPAR run (Fig. 7 bridging effect).

    A committed shared point adds a forward edge from the EC-OP net to
    the EO-CP net (the EC-OP value now drives/controls the EO-CP line
    through the shared point), which must be visible to Rule 1 checks
    on later candidate pairs.
    """

    def __init__(self, instances):
        self.net_to_sinks, self.net_to_driver_port = \
            build_fanout_graph(instances)
        # synthetic edges from committed shared points: net -> [net, ...]
        self.synthetic_edges = {}

    def commit_shared_point(self, eo_cp_net, ec_op_net):
        """Record that a shared point now connects ec_op_net -> eo_cp_net."""
        self.synthetic_edges.setdefault(ec_op_net, []).append(eo_cp_net)

    def forward_nets(self, net, instances):
        """
        Return the set of nets immediately reachable forward from `net`:
        one hop through real gates (via net_to_sinks + each consuming
        instance's output nets) plus any synthetic shared-point edges.
        """
        result = set()

        for (inst_idx, _ip) in self.net_to_sinks.get(net, []):
            inst = instances[inst_idx]
            base = COP.get_cell_base(inst['cell'])
            info = COP.CELL_OUTPUT_PORTS.get(base)
            if info is None:
                continue
            _in_ports, out_ports = info
            for op in out_ports:
                onet = inst['conn'].get(op)
                if onet:
                    result.add(onet)

        for onet in self.synthetic_edges.get(net, []):
            result.add(onet)

        return result


# ---------------------------------------------------------------------------
# RULE 1 -- COMBINATIONAL LOOP GENERATION
# ---------------------------------------------------------------------------

def is_reachable(start_net, target_net, graph, instances, stop_at_scan=True):
    """
    BFS forward from start_net. Returns True if target_net is reachable
    through combinational logic (and, per the paper's loop-bridging case,
    through any already-committed synthetic shared-point edges).

    stop_at_scan: if True, do not propagate through a flip-flop's D->Q/QN
    (sequential elements break combinational reachability for Rule 1
    purposes -- a path through a register is not a combinational loop).
    """
    if start_net == target_net:
        return True

    visited = set([start_net])
    queue = deque([start_net])

    ff_bases = ('SDFFX1', 'SDFFX2', 'DFFX1')

    while queue:
        net = queue.popleft()

        for (inst_idx, _ip) in graph.net_to_sinks.get(net, []):
            inst = instances[inst_idx]
            base = COP.get_cell_base(inst['cell'])

            if stop_at_scan and base in ff_bases:
                continue  # do not cross sequential elements

            info = COP.CELL_OUTPUT_PORTS.get(base)
            if info is None:
                continue
            _in_ports, out_ports = info
            for op in out_ports:
                onet = inst['conn'].get(op)
                if onet and onet not in visited:
                    if onet == target_net:
                        return True
                    visited.add(onet)
                    queue.append(onet)

        for onet in graph.synthetic_edges.get(net, []):
            if onet not in visited:
                if onet == target_net:
                    return True
                visited.add(onet)
                queue.append(onet)

    return False


def check_rule1(eo_cp_net, ec_op_net, graph, instances):
    """
    Rule 1: A shared point cannot be inserted between an EO-CP line and
    an EC-OP line which is located in the fanout cone of the
    corresponding EO-CP line.

    Returns True if the pair is SAFE to insert (no loop), False if
    forbidden (ec_op_net is reachable from eo_cp_net -> would create a
    combinational loop once the shared point's feedback path exists).
    """
    reachable = is_reachable(eo_cp_net, ec_op_net, graph, instances)
    return not reachable


# ---------------------------------------------------------------------------
# RULE 2 -- FAULT-PROPAGATION BLOCK
# ---------------------------------------------------------------------------

# A gate's output is forced to a known value by ONE input being set,
# regardless of the other inputs, exactly when that input value is
# "dominant" for the gate's function. For simple 2-4 input AND/OR-family
# gates, dominance doesn't depend on which port receives the value
# (every input is symmetric). For 3-input compound gates of the form
# AOI21/OAI21/AO21/OA21, dominance is ASYMMETRIC: only the "A3" port
# (the one OR'd/AND'd in on top of the inner AND/OR pair) is dominant;
# A1/A2 alone never determine the output since they only fix one side
# of an inner pair whose partner is still unknown. See
# compound_dominance_notes.py for the per-gate derivation.
AND_FAMILY = ('AND2', 'AND3', 'AND4', 'NAND2', 'NAND3', 'NAND4')
OR_FAMILY  = ('OR2', 'OR3', 'OR4', 'NOR2', 'NOR3', 'NOR4')
INVERTING  = ('NAND2', 'NAND3', 'NAND4', 'NOR2', 'NOR3', 'NOR4',
            'INV', 'AOI21', 'AOI22', 'AOI221', 'AOI222',
            'OAI21', 'OAI22', 'OAI221', 'OAI222')

# Gates where exactly ONE specific port is dominant, and the value that
# dominates it. Y = ~((A1&A2)|A3): A3=1 forces Y=0 (pre-inversion out=1).
# Y = ~((A1|A2)&A3): A3=0 forces Y=1 (pre-inversion out=0).
# Y = (A1&A2)|A3:     A3=1 forces Y=1 (pre-inversion out=1).
# Y = (A1|A2)&A3:     A3=0 forces Y=0 (pre-inversion out=0).
ASYMMETRIC_DOMINANT = {
    'AOI21': {'A3': (1, 1)},   # (dominant input value, pre-inversion output)
    'OAI21': {'A3': (0, 0)},
    'AO21':  {'A3': (1, 1)},
    'OA21':  {'A3': (0, 0)},
}

# Gates with NO single-input dominance: every AND/OR sub-term needs
# BOTH of its own inputs known before it resolves, so a single injected
# value never determines the output on its own. Listed explicitly
# (rather than silently falling through) so it's clear this is a
# verified structural fact, not an unhandled gap.
NO_DOMINANCE_GATES = (
    'AOI22', 'AOI221', 'AOI222',
    'OAI22', 'OAI221', 'OAI222',
    'AO22', 'AO221', 'AO222',
    'OA22', 'OA221', 'OA222',
)

# MUX21/MUX41: dominance doesn't apply in the simple sense -- a select
# line determines WHICH data input matters but not the output VALUE,
# and a data input alone is irrelevant without a known select line.
# TODO: implement select-line-aware propagation; for now these are
# conservatively treated as having no dominant port (propagation always
# stops here), which only under-propagates, never over-propagates.
MUX_GATES = ('MUX21', 'MUX41')


def dominant_value(base, port=None):
    """
    Return the input value that forces this gate's output regardless of
    other inputs, given the value arrives on `port`, or None if this
    (gate, port) combination has no dominance.

    For simple AND/OR-family gates, `port` is irrelevant (any input is
    symmetric). For asymmetric compound gates (AOI21/OAI21/AO21/OA21),
    only a specific port is dominant. For gates with no single-input
    dominance, or MUX gates, always returns None.
    """
    if base in AND_FAMILY:
        return 0
    if base in OR_FAMILY:
        return 1
    if base in NO_DOMINANCE_GATES or base in MUX_GATES:
        return None
    if base in ASYMMETRIC_DOMINANT:
        entry = ASYMMETRIC_DOMINANT[base].get(port)
        if entry is not None:
            return entry[0]
        return None
    return None


def forced_output(base, dom_val, port=None):
    """Given a dominant input value just applied on `port`, what output
    does it force?"""
    is_inv = base in INVERTING
    if base in ASYMMETRIC_DOMINANT and port in ASYMMETRIC_DOMINANT[base]:
        _dv, pre_inv_out = ASYMMETRIC_DOMINANT[base][port]
        out = pre_inv_out
    elif base in AND_FAMILY:
        # dom_val must be 0 here (the only dominant value for AND-family)
        out = 0
    else:
        # OR-family, dom_val must be 1
        out = 1
    return (1 - out) if is_inv else out


FF_BASES = ('SDFFX1', 'SDFFX2', 'DFFX1')


def propagate_forward_injection(start_net, inject_val, graph, instances,
                                stop_nets, convergence_nets=frozenset()):
    """
    Inject `inject_val` onto start_net and propagate forward, setting
    controllability=1 or 0 on each net the value deterministically
    reaches. Stops at scan elements (flip-flops) and at any net in
    `stop_nets` (used to keep the propagation from crossing back over
    the EC-OP/EO-CP boundary lines themselves).

    convergence_nets: nets belonging to the EO-CP's blocked cone (e.g.
        ConeG6_out in the paper's Fig. 9). Any gate that has one of
        these nets as an input is treated as a forced stopping point
        REGARDLESS of dominance, because that is exactly the gate where
        the propagating value meets the blocked-cone signal and must
        be checked by the backward observability pass -- pushing the
        forced value further downstream (past that gate's output)
        would skip the check entirely. This matches Fig. 9(a): the
        bold/propagating path halts at G4's INPUT even though G4's
        output is dominantly determined (G2's output=0 dominates the
        AND2), because G4 is where ConeG6_out converges.

    Returns:
        forced_cc   : dict net -> forced value (0 or 1) for nets reached
        last_gates  : list of inst_idx where propagation stopped --
                      either because the value did not dominate the
                      gate, or because the gate is a convergence point
                      with the blocked cone. These are the backward-
                      pass starting points.
    """
    forced_cc = {start_net: inject_val}
    last_gates = []
    visited_nets = set([start_net])
    queue = deque([start_net])

    while queue:
        net = queue.popleft()
        val = forced_cc[net]

        for (inst_idx, ip) in graph.net_to_sinks.get(net, []):
            inst = instances[inst_idx]
            base = COP.get_cell_base(inst['cell'])

            if base in FF_BASES:
                continue  # do not propagate through scan elements

            info = COP.CELL_OUTPUT_PORTS.get(base)
            if info is None:
                continue
            in_ports, out_ports = info
            conn = inst['conn']

            # Forced stop: this gate also has a blocked-cone net as one
            # of its inputs -- halt here regardless of dominance so the
            # backward pass can check observability at this exact gate.
            has_convergence_input = any(
                conn.get(p) in convergence_nets for p in in_ports)
            if has_convergence_input:
                last_gates.append(inst_idx)
                continue

            if base in ('INV', 'BUF'):
                out_val = (1 - val) if base == 'INV' else val
                for op in out_ports:
                    onet = conn.get(op)
                    if onet and onet not in stop_nets and onet not in visited_nets:
                        forced_cc[onet] = out_val
                        visited_nets.add(onet)
                        queue.append(onet)
                continue

            dom = dominant_value(base, port=ip)
            if dom is not None and val == dom:
                out_val = forced_output(base, dom, port=ip)
                for op in out_ports:
                    onet = conn.get(op)
                    if onet and onet not in stop_nets and onet not in visited_nets:
                        forced_cc[onet] = out_val
                        visited_nets.add(onet)
                        queue.append(onet)
            else:
                # value reached this gate but did not dominate it:
                # output unknown -> this gate is a stopping point
                last_gates.append(inst_idx)


        for onet in graph.synthetic_edges.get(net, []):
            if onet not in stop_nets and onet not in visited_nets:
                forced_cc[onet] = val
                visited_nets.add(onet)
                queue.append(onet)

    return forced_cc, last_gates


def observability_from_gates(start_gates, forced_cc, graph, instances,
                            cc1_vals, co_vals, stop_nets, max_hops=200):
    """
    Compute observability backward starting from `start_gates` (instance
    indices), using `forced_cc` where available and falling back to the
    existing global cc1_vals for any input not covered by the forward
    injection. Does not cross back past nets in `stop_nets` (EC-OP line,
    EO-CP line) or past scan elements.

    Returns: dict net -> co value, restricted to nets visited in this
    bounded backward pass.
    """
    local_co = {}

    # Seed: the *existing* CO of each start gate's output (from the
    # already-computed global co_vals) is the observability budget
    # available to propagate backward from that gate.
    queue = deque()
    for inst_idx in start_gates:
        inst = instances[inst_idx]
        base = COP.get_cell_base(inst['cell'])
        info = COP.CELL_OUTPUT_PORTS.get(base)
        if info is None:
            continue
        _in_ports, out_ports = info
        for op in out_ports:
            onet = inst['conn'].get(op)
            if onet:
                seed = co_vals.get(onet, 0.0)
                if local_co.get(onet, 0.0) < seed:
                    local_co[onet] = seed
                queue.append((inst_idx, onet))

    hops = 0
    while queue and hops < max_hops:
        hops += 1
        inst_idx, onet = queue.popleft()
        inst = instances[inst_idx]
        base = COP.get_cell_base(inst['cell'])

        if base in FF_BASES:
            continue

        info = COP.CELL_OUTPUT_PORTS.get(base)
        if info is None:
            continue
        in_ports, out_ports = info
        conn = inst['conn']

        input_cc1 = {}
        for ip in in_ports:
            net = conn.get(ip)
            if net:
                input_cc1[ip] = forced_cc.get(net, cc1_vals.get(net, 0.5))

        co_outputs = {}
        for op in out_ports:
            net = conn.get(op)
            if net:
                co_outputs[op] = local_co.get(net, co_vals.get(net, 0.0))

        results = COP.compute_co_inputs(base, input_cc1, co_outputs)

        for ip, val in results.items():
            net = conn.get(ip)
            if not net:
                continue
            # Always RECORD the observability value that lands on this
            # net, even if it's a stop_net (e.g. the blocked-cone net
            # itself, which is exactly what the caller needs to read).
            # Use a presence check rather than a strict-greater epsilon
            # comparison, so that a legitimate value of 0.0 (e.g. an
            # AND-gate's sibling input being fully unobservable) still
            # gets recorded on first visit instead of being silently
            # dropped because 0.0 > 0.0 + eps is False.
            if net not in local_co:
                local_co[net] = val
            elif val > local_co[net] + 1e-9:
                local_co[net] = val
            # But do NOT walk further backward through a stop_net's own
            # driver -- that would cross back over the EC-OP/EO-CP
            # boundary, which Section V-B explicitly forbids ("moving
            # does not proceed beyond the current EC-OP line, EO-CP
            # line, and scan elements").
            if net in stop_nets:
                continue
            drv = graph.net_to_driver_port.get(net)
            if drv:
                d_idx, _d_port = drv
                d_base = COP.get_cell_base(instances[d_idx]['cell'])
                if d_base not in FF_BASES:
                    queue.append((d_idx, net))

    return local_co


def check_rule2(eo_cp_net, ec_op_net, eo_cp_required_value, graph,
                instances, cc1_vals, co_vals, blocked_cone_nets):
    """
    Rule 2: An EO-CP cannot be paired with an EC-OP that blocks fault
    propagation within blocked cones.

    eo_cp_required_value: the control value (0 or 1) the EO-CP line
        needs in order to propagate faults (e.g. 1 for an OR-type CP,
        0 for an AND-type CP -- see Section III-A / Fig. 1(a)).
    blocked_cone_nets: the set of net(s) whose observability must stay
        nonzero for the pairing to be considered safe (the EO-CP's
        original blocked cone output, e.g. ConeG6's output net).

    Returns:
        dict with keys 'direct' and 'inverted', each True (safe to use
        that control scheme) or False (blocked).
    """
    stop_nets = set([eo_cp_net, ec_op_net])
    results = {}

    for scheme, inject_val in (('direct', eo_cp_required_value),
                                ('inverted', 1 - eo_cp_required_value)):
        forced_cc, last_gates = propagate_forward_injection(
            ec_op_net, inject_val, graph, instances, stop_nets,
            convergence_nets=blocked_cone_nets)

        local_co = observability_from_gates(
            last_gates, forced_cc, graph, instances,
            cc1_vals, co_vals, stop_nets)

        blocked = any(local_co.get(n, co_vals.get(n, 0.0)) <= 1e-9
                    for n in blocked_cone_nets)
        results[scheme] = not blocked

    return results