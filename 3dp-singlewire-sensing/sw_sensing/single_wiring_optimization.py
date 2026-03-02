"""
Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International Public License
https://creativecommons.org/licenses/by-nc-sa/4.0/legalcode

Copyright (c) 2025, Takanori Fujiwara and S. Sandra Bae
All rights reserved.
"""

import lcapy
import sympy as sy
import numpy as np


def generate_circuit(resistances, node_names, touch_point=None):
    # the part after semicolons are only related to schematics
    cct = lcapy.Circuit()
    cct.add("V pin5 0 step 5.0; down=6, cground")
    cct.add("W pin5 pin5_2")
    cct.add(f"R0 pin5_2 in {resistances[0]}; down")
    cct.add(f"W in {node_names[0]}; right=2")
    for i in range(1, len(resistances)):
        cct.add(f"R{i} {node_names[i-1]} {node_names[i]} {resistances[i]}; down")
    cct.add(f"W {node_names[-1]} out; down")
    cct.add(f"W in pin2; down=4")
    cct.add("Rx pin2 0 100.0e6; down, cground")
    if touch_point is not None:
        cct.add(f"W {touch_point} touch; right=2")
        cct.add(f"C touch 0 100e-12; down, ground")
    cct.add("; scale=0.5")
    return cct


def substitute_symbols(f, symbol_val_dict):
    substitutes = []
    for symbol in f.free_symbols:
        if symbol.name in symbol_val_dict:
            substitutes.append((symbol, symbol_val_dict[symbol.name]))
    return f.subs(substitutes)


def gen_eval_one_circuit(r0, r1, voltage_thres=2.5):
    def eval_one_circuit(transient_func):
        eq = substitute_symbols(
            sy.Eq(transient_func, voltage_thres), {"r0": r0, "r1": r1}
        )
        for symbol in transient_func.free_symbols:
            if symbol.name == "t":
                tsymbol = symbol
        ans = sy.solveset(eq, tsymbol, domain=sy.core.S.Reals)
        if len(ans.args) == 0:
            t_thres = np.nan  # no answer
        else:
            t_thres = ans.args[0]
        return t_thres

    return eval_one_circuit


def optimize_resistances(
    n_nodes,
    wire_resistance_candidates=np.arange(0.1e6, 10e6, 200e3),
    trace_resistance_candidates=np.arange(100e3, 500e3, 50e3),
    voltage_thres=2.5,
    time0_voltage_thres=2.5 * 0.9,  # avoid too close to voltage_thres
    timeout_thres=1e-3,  # avoid passing reasonable time
    verbose=False,
):
    # we only need to care about circuits that reach voltage_thres slowest and fastest
    slowest_cct = generate_circuit(
        ["r0", "r1"], ["node_touch", "node_other"], "node_touch"
    )
    fastest_cct = generate_circuit(
        ["r0", "r1"], ["node_other", "node_touch"], "node_touch"
    )

    slowest_transf = sy.simplify(slowest_cct.pin2.V.transient_response().sympy)
    fastest_transf = sy.simplify(fastest_cct.pin2.V.transient_response().sympy)

    best_score = 0
    best_r0 = wire_resistance_candidates[0]
    best_r = trace_resistance_candidates[0]
    _wire_resistance_candidates = np.array(wire_resistance_candidates)
    for r in trace_resistance_candidates:
        r1 = r * (n_nodes - 1)
        r0_min = r1
        r0_candidates = _wire_resistance_candidates[
            _wire_resistance_candidates > r0_min
        ]
        for r0 in r0_candidates:
            if verbose:
                print("r0", r0, "r", r, "best score", best_score)
            if r < best_r:
                # when increasing r0, r1 only satisfies the condition when over best_r
                continue
            time0_voltage_slowest = substitute_symbols(
                slowest_transf, {"t": timeout_thres * 1e-6, "r0": r0, "r1": r1}
            )
            time0_voltage_fastest = substitute_symbols(
                fastest_transf, {"t": timeout_thres * 1e-6, "r0": r0, "r1": r1}
            )
            if max(time0_voltage_slowest, time0_voltage_fastest) > time0_voltage_thres:
                score = 0  # constraint violation
            else:
                eval_one_circuit = gen_eval_one_circuit(r0, r1, voltage_thres)
                t_thres_slowest = eval_one_circuit(slowest_transf)
                t_thres_fastest = eval_one_circuit(fastest_transf)
                if max(t_thres_slowest, t_thres_fastest) > timeout_thres:
                    score = 0  # constraint violation
                else:
                    score = np.abs(t_thres_slowest - t_thres_fastest)
                if (score == sy.nan) or (score == np.nan):
                    score = 0  # no answer
            if score > best_score:
                best_score = score
                best_r0 = r0
                best_r = r
                # once cost becomes not nan, smallest r1 is the best. so break
                break
    return best_r0, best_r, best_score


# def optimize_resistances(
#     n_nodes,
#     wire_resistance_candidates=np.arange(0.1e6, 10e6, 200e3),
#     trace_resistance_candidates=np.arange(100e3, 500e3, 50e3),
#     voltage_thres=2.5,
#     time0_voltage_thres=2.5 * 0.9,  # avoid too close to voltage_thres
#     timeout_thres=1e-3,
# ):  # avoid passing reasonable time
#     nodes = [f"node{i}" for i in range(n_nodes)]
#     resistances = ["r0"] + ["r1"] * (n_nodes - 1)

#     # symbolic voltage transients (instead of actual values using 'r0' and 'r1')
#     symbolic_v_transients = []
#     for touch_node in nodes:
#         cct = generate_circuit(resistances, nodes, touch_node)
#         symbolic_v_transients.append(sy.simplify(cct.pin2.V.transient_response().sympy))

#     # optimization
#     # probably, we only need to care about circuits that reach voltage_thres slowest and fastest
#     transf1 = symbolic_v_transients[0]  # usually slowest to reach
#     transf2 = symbolic_v_transients[-1]  # usually fastest

#     best_score = 0
#     best_r0 = wire_resistance_candidates[0]
#     best_r1 = trace_resistance_candidates[0]
#     for r0 in wire_resistance_candidates:
#         for r1 in trace_resistance_candidates:
#             print("r0", r0, "r1", r1, "best score", best_score)
#             if r1 < best_r1:
#                 # when increasing r0, r1 only satisfies the condition when over best_r1
#                 continue
#             time0_voltage1 = substitute_symbols(
#                 transf1, {"t": timeout_thres * 1e-6, "r0": r0, "r1": r1}
#             )
#             time0_voltage2 = substitute_symbols(
#                 transf2, {"t": timeout_thres * 1e-6, "r0": r0, "r1": r1}
#             )
#             if max(time0_voltage1, time0_voltage2) > time0_voltage_thres:
#                 score = 0  # constraint violation
#             else:
#                 eval_one_circuit = gen_eval_one_circuit(r0, r1, voltage_thres)
#                 t_thres1 = eval_one_circuit(transf1)
#                 t_thres2 = eval_one_circuit(transf2)
#                 if max(t_thres1, t_thres2) > timeout_thres:
#                     score = 0  # constraint violation
#                 else:
#                     score = np.abs(t_thres1 - t_thres2)
#                 if (score == sy.nan) or (score == np.nan):
#                     score = 0  # no answer
#             if score > best_score:
#                 best_score = score
#                 best_r0 = r0
#                 best_r1 = r1
#                 # once cost becomes not nan, smallest r1 is the best. so break
#                 break
#     return best_r0, best_r1, best_score
