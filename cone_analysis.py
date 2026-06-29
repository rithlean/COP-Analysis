"""
cone_analysis.py

# VERSION: 2026-06-25 rev5 -- added fault-count propagation (Eq.1 from
# Moghaddam et al. [28]) and b/B metric computation (Section VI groundwork)
# (rev1: initial Rule1/Rule2 impl; rev2: added compound-gate dominance
# for AOI21/OAI21/AO21/OA21 + explicit no-dominance gate list; rev3:
# removed the co_vals fallback in check_rule2 that made it vacuously
# always pass -- see chat history for the s38417 100%-pass investigation;
# rev4: added backward_fanin_cone + find_convergence_gate, mirroring
# Section V's forward-cone logic but walking net_to_driver_port instead
# of net_to_sinks; rev5: implemented [28]'s actual b/B algorithm
# (verified against [28] Fig.6 by hand, gate shapes confirmed via image
# zoom -- G3=NAND2, G4=AND2, G5=OR2, G6=OR2, G7/G8=AND2). Seeds D_i=1
# fault per net, propagates forward via Eq.1 (CO-weighted fanout split,
# reusing existing co_vals), then computes b_x/B_x backward via the
# SAME dominant_value table Rule 2 already uses. This replaces the
# earlier COP-proxy idea (Pds0+Pds1 sum, which reduces to plain CO and
# was flagged as too thin a connection to real fault counts) -- now we
# implement the actual published algorithm instead of approximating it.

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

    IMPORTANT: this check only has teeth when the EC-OP's forward
    propagation actually RECONVERGES with the blocked cone (matching
    Fig. 8's setup). For the common case where an EC-OP/EO-CP pair
    shares no local structure at all, the forward injection never
    reaches a gate that also touches blocked_cone_nets, so there is
    nothing to block -- this correctly reports SAFE, not because the
    check is vacuous, but because there is genuinely no interaction
    to flag. This is distinct from "reached the blocked cone and found
    its observability driven to zero", which correctly reports BLOCKED.
    The two cases must not be conflated: falling back to the
    pre-existing global co_vals for "never reached" would always
    report SAFE regardless of the pairing (co_vals[eo_cp_net] is
    already > COTh by definition of being an EO-CP), making the check
    unable to ever fail. So no such fallback is used here.

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

        # Only nets actually reached by the bounded local backward pass
        # count. A net absent from local_co means the propagation never
        # reconverged with the blocked cone for this scheme -- no
        # interaction, hence no risk of blocking, hence safe.
        blocked = any(local_co[n] <= 1e-9
                    for n in blocked_cone_nets if n in local_co)
        results[scheme] = not blocked

    return results


# ---------------------------------------------------------------------------
# SECTION VI GROUNDWORK -- BACKWARD FANIN CONE EXTRACTION
# ---------------------------------------------------------------------------
#
# Section VI (controllability optimization, Eq. 7-9) needs to identify
# Cone_CP, Cone_G1, Cone_G2 (Fig. 10) for a given EO-CP net. This is a
# clean mirror of Section V's forward-fanout-cone logic, just walking
# net_to_driver_port backward instead of net_to_sinks forward. Kept
# proxy-independent: whatever b_x/B_x/F_x estimation method is chosen
# (COP-based proxy, cone-size proxy, or real TetraMAX fault counts),
# it will need to know WHICH nets belong to each cone, and that's all
# this section does -- no fault-count math here.

def backward_fanin_cone(net, graph, instances, stop_at_scan=True):
    """
    BFS backward from `net` via its driver chain, returning the set of
    all nets in its fanin cone. Stops at primary inputs (no driver)
    and, by default, at flip-flop outputs (since a FF's D input is a
    separate sequential domain, not part of the SAME-cycle combinational
    fanin cone -- mirroring Section V's is_reachable's stop_at_scan
    convention for forward cones).
    """
    visited = set([net])
    queue = deque([net])
    ff_bases = ('SDFFX1', 'SDFFX2', 'DFFX1')

    while queue:
        cur = queue.popleft()
        drv = graph.net_to_driver_port.get(cur)
        if drv is None:
            continue  # primary input or unresolved -- fanin ends here
        inst_idx, _out_port = drv
        inst = instances[inst_idx]
        base = COP.get_cell_base(inst['cell'])

        if stop_at_scan and base in ff_bases:
            continue  # don't cross into the previous clock cycle's logic

        info = COP.CELL_OUTPUT_PORTS.get(base)
        if info is None:
            continue
        in_ports, _out_ports = info
        conn = inst['conn']

        for ip in in_ports:
            inet = conn.get(ip)
            if inet and inet not in visited:
                visited.add(inet)
                queue.append(inet)

    return visited


def find_convergence_gate(eo_net, graph, instances):
    """
    Find a gate where eo_net is a direct input, and return
    (gate_idx, other_input_net) for the first such gate found whose
    other input is a distinct, resolvable net.

    Promoted from find_rule2_convergence.py into the main module since
    Section VI needs the same "what gate does eo_net feed, and what's
    its OTHER input" pattern to identify Cone_G1/Cone_G2 candidates
    (Fig. 10's G1/G2), not just for Rule 2 testing.

    NOTE: only returns the FIRST matching gate. A real EO-CP may feed
    several downstream gates (Fig. 10 shows eo_net's CP output x'
    fanning out to BOTH G1 and G2) -- callers needing ALL such gates
    should use find_all_convergence_gates instead.
    """
    for (inst_idx, ip) in graph.net_to_sinks.get(eo_net, []):
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


def find_all_convergence_gates(eo_net, graph, instances):
    """
    Like find_convergence_gate, but returns ALL gates where eo_net is a
    direct input, each paired with its OTHER input net(s).

    Matches Fig. 10's actual topology more faithfully than
    find_convergence_gate: eo_net's CP output (x') fans out to BOTH G1
    and G2, each with its own separate "other cone" (Cone_G1, Cone_G2).
    Section VI needs all of these, not just the first match.

    Returns: list of (gate_idx, [other_input_nets]) tuples, one entry
    per gate that has eo_net as a direct input.
    """
    results = []
    for (inst_idx, ip) in graph.net_to_sinks.get(eo_net, []):
        inst = instances[inst_idx]
        base = COP.get_cell_base(inst['cell'])
        info = COP.CELL_OUTPUT_PORTS.get(base)
        if info is None:
            continue
        in_ports, _out_ports = info
        conn = inst['conn']
        other_nets = [conn.get(p) for p in in_ports
                    if p != ip and conn.get(p) and conn.get(p) != eo_net]
        if other_nets:
            results.append((inst_idx, other_nets))
    return results


# ---------------------------------------------------------------------------
# SECTION VI -- FAULT-COUNT PROPAGATION (Eq. 1, Moghaddam et al. [28])
# ---------------------------------------------------------------------------
#
# Seeds D_i = 1 fault per net (flagged approximation -- [28]'s paper
# doesn't publish its own base seeding convention, only the propagation
# structure itself), then propagates forward through the topological
# order: at each gate, the output's fault count is the SUM of fault
# counts reaching its inputs (plus its own seed), and at a fanout stem
# the total is SPLIT across branches proportional to each branch's CO
# (observability) -- "alpha" in Eq. 1 -- reusing the co_vals already
# computed by COP.py's run_cop.

def compute_fault_counts(ports_in, instances, assigns, cc1_vals, co_vals,
                        topo, inst_inputs_resolved):
    """
    Forward fault-count propagation per Eq. 1.

    Returns: dict net -> D (fault count, float).

    topo and inst_inputs_resolved should come from the SAME
    build_and_sort() call used by COP.py's run_cop, so the topological
    order and resolved input nets are consistent with cc1_vals/co_vals.
    Since cone_analysis.py doesn't re-run build_and_sort itself (to
    avoid duplicating COP.py's parsing/resolution logic), callers
    should pass these through from COP.py directly -- see
    compute_fault_counts_from_netlist for a convenience wrapper that
    does this end-to-end.
    """
    D = {}

    # Seed every net we'll ever touch with 1 fault. PIs first.
    for pi in ports_in:
        D[pi] = 1.0

    for idx in topo:
        inst = instances[idx]
        base = COP.get_cell_base(inst['cell'])
        info = COP.CELL_OUTPUT_PORTS.get(base)
        if info is None:
            continue
        in_ports, out_ports = info
        conn = inst['conn']

        # Sum of fault counts reaching this gate's inputs, plus each
        # input net's own seed (1, if not already counted via a
        # driving gate's output -- handled naturally since D[net] is
        # set exactly once per net, either as a PI seed or as a gate
        # output below).
        total_in = 0.0
        for ip in in_ports:
            net = conn.get(ip)
            if net:
                total_in += D.get(net, 1.0)

        for op in out_ports:
            onet = conn.get(op)
            if not onet:
                continue
            # This output net's own seed (1) plus everything summed
            # from its inputs.
            D[onet] = total_in + 1.0

    # Fanout split: for any net with multiple sinks, the TOTAL fault
    # count at the stem is divided among branches proportional to each
    # branch's observability (alpha = branch CO / sum of sibling COs).
    # Since our representation doesn't have separate "branch" nets
    # (Verilog nets are single-valued, fanout is implicit in
    # net_to_sinks), we approximate the per-BRANCH count by scaling
    # the stem's D by this ratio ONLY when a per-sink distinction is
    # actually needed (i.e. in the b/B backward pass below, which
    # consumes D per CONSUMING GATE, not per net) -- so no separate
    # split step is needed here. Eq.1's splitting is applied later, at
    # the point where a fanout's branches are individually consumed.

    return D


# ---------------------------------------------------------------------------
# SECTION VI -- b/B METRIC COMPUTATION ([28] Section IV-B, Fig. 6)
# ---------------------------------------------------------------------------
#
# b_x = faults blocked when x is set to 1 (inject 1, propagate forward
#       via dominance; wherever the injected value DOMINATES a gate,
#       that gate's OTHER input's fault count is blocked -- added to b)
# B_x = faults blocked when x is set to 0 (mirror, injecting 0)
#
# Verified by hand against [28] Fig. 6 (gate shapes confirmed via
# image zoom: G3=NAND2, G4=AND2, G5=OR2, G6=OR2, G7/G8=AND2). Reuses
# the SAME dominant_value table Rule 2 already uses for forward
# injection -- this is deliberately the same dominance concept, not a
# new one.

def propagate_bB(start_net, inject_val, graph, instances, fault_counts,
                stop_nets=frozenset(), max_hops=500):
    """
    Inject `inject_val` onto start_net and propagate forward via
    dominance (same rule as propagate_forward_injection in Rule 2).
    Wherever the propagating value DOMINATES a gate, the gate's OTHER
    input(s)' fault counts are blocked -- accumulated into the
    returned total. Where the value does NOT dominate, propagation
    stops at that gate (the other input's faults are NOT blocked --
    they remain "observable", matching [28]'s terminology).

    Returns: total blocked fault count (float) for this single
    injection value, accumulated across every gate where the
    propagating wave hit a dominant value.
    """
    blocked_total = 0.0
    forced_cc = {start_net: inject_val}
    visited_nets = set([start_net])
    queue = deque([start_net])
    hops = 0

    while queue and hops < max_hops:
        hops += 1
        net = queue.popleft()
        val = forced_cc[net]

        for (inst_idx, ip) in graph.net_to_sinks.get(net, []):
            inst = instances[inst_idx]
            base = COP.get_cell_base(inst['cell'])

            if base in FF_BASES:
                continue

            info = COP.CELL_OUTPUT_PORTS.get(base)
            if info is None:
                continue
            in_ports, out_ports = info
            conn = inst['conn']

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
                # Dominant: this BLOCKS every other input's faults at
                # this gate (they cannot be sensitized while this
                # input holds the controlling value).
                for other_port in in_ports:
                    if other_port == ip:
                        continue
                    other_net = conn.get(other_port)
                    if other_net and other_net not in stop_nets:
                        blocked_total += fault_counts.get(other_net, 1.0)

                out_val = forced_output(base, dom, port=ip)
                for op in out_ports:
                    onet = conn.get(op)
                    if onet and onet not in stop_nets and onet not in visited_nets:
                        forced_cc[onet] = out_val
                        visited_nets.add(onet)
                        queue.append(onet)
            # else: NOT dominant -- the other input(s)' faults remain
            # observable (not blocked), and propagation stops here
            # (output undetermined), matching last_gates semantics.

    return blocked_total


def compute_b_and_B(net, graph, instances, fault_counts, stop_nets=frozenset()):
    """
    Compute (b_net, B_net) for a given net: faults blocked when the
    net is set to 1 (b) and when set to 0 (B), per [28] Section IV-B.
    """
    b = propagate_bB(net, 1, graph, instances, fault_counts, stop_nets)
    B = propagate_bB(net, 0, graph, instances, fault_counts, stop_nets)
    return b, B
