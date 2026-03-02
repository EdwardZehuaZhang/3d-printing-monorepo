import sympy as sy
import numpy as np
from sw_sensing.geometry import trace_filling

# two-wire condition
# r_p+1 > np.log(2) * 5e-6 / 100e-12
dy = 1.2
dz = 1.2
# fill_area_radius = 2.5  # => 5mm diameter
fill_area_radius = 5.0  # => 10mm diameter
h_resistance_per_length = 256  # per mm

min_diff_thres = 5e-6  # 5microsec
aiming_trace_resistance_dbl = np.log(2) * 5e-6 / 100e-12
aiming_h_trace_length_dbl = aiming_trace_resistance_dbl / h_resistance_per_length

angles = [0, 15, 30, 45, 60, 75, 90]
for angle in angles:
    for length in np.arange(5, 25, 1):
        y = sy.N(length * sy.cos((angle / 180) * sy.pi))
        z = sy.N(length * sy.sin((angle / 180) * sy.pi))

        # print(y, z)
        line_positions = np.array(
            [[0, 0, 0], [0, y / 2, z / 2], [0, y, z]], dtype=float
        )
        # line_positions = np.array([[0, 0, 0], [0, 0, length / 2], [0, 0, length]])

        trace = trace_filling(
            line_positions=line_positions,
            fill_area_radius=fill_area_radius,
            dy=1.2,
            dz=1.2,
        )

        h_trace_length = 0.0
        v_trace_length = 0.0
        for s_pos, t_pos in zip(trace[:-1], trace[1:]):
            vec = t_pos - s_pos
            mag = np.linalg.norm(vec)
            if (vec[2] > 0) or (vec[2] < 0):
                # vertical line
                v_trace_length += mag
            else:
                # horizontal line
                h_trace_length += mag
        if h_trace_length > aiming_h_trace_length_dbl:
            break
    print(angle, length, h_trace_length, v_trace_length)
double_length = length

# one wire condition
import pandas as pd
from sw_sensing.single_wiring_optimization import (
    generate_circuit,
    gen_eval_one_circuit,
    substitute_symbols,
)
from pathos.multiprocessing import ProcessPool as Pool
from scipy.spatial.distance import pdist

voltage_thres = 2.5
time0_voltage_thres = voltage_thres  # * 0.9

n_nodes_list = range(2, 31)
trace_r_candidates = np.arange(10e3, 1000e3, 2e3)  # resistor trace resistance values

results = []
for n_nodes in n_nodes_list:
    resistances = ["r0"] + ["r1"] * (n_nodes - 1)
    symbolic_v_transients = []
    connect_names = [f"Node{i}" for i in range(n_nodes)]
    for touch_point in connect_names:  # removed None
        cct = generate_circuit(resistances, connect_names, touch_point)
        symbolic_v_transients.append(sy.simplify(cct.pin2.V.transient_response().sympy))

    for r1 in trace_r_candidates:
        wire_r_candidates = np.arange(
            r1 * (n_nodes - 1), r1 * (n_nodes - 1) * 3, 0.1e6
        )  # wire resistance values
        for r0 in wire_r_candidates:
            # constraint violation check
            transf1 = symbolic_v_transients[0]  # usually slowest to reach
            transf2 = symbolic_v_transients[-1]  # usually fastest
            time0_voltage1 = substitute_symbols(
                transf1, {"t": 1e-9, "r0": r0, "r1": r1}
            )
            time0_voltage2 = substitute_symbols(
                transf2, {"t": 1e-9, "r0": r0, "r1": r1}
            )
            if max(time0_voltage1, time0_voltage2) > time0_voltage_thres:
                min_diff = 0
                continue

            eval_one_circuit = gen_eval_one_circuit(r0, r1, voltage_thres)
            with Pool() as p:
                t_thresholds = p.map(eval_one_circuit, symbolic_v_transients)
            t_thresholds = np.array(t_thresholds, dtype=float)
            min_diff = pdist(t_thresholds[:, None], "minkowski", p=1).min()

            print(r0, r1, min_diff)

            if min_diff >= min_diff_thres:
                # print(r0, r1, min_diff)
                break
        if min_diff >= min_diff_thres:
            results.append(
                {
                    "n_nodes": n_nodes,
                    "wire_resistance": r0,
                    "trace_resistance": r1,
                    "min_t_diff": min_diff,
                }
            )
            break


angle = 15
length_min = 5
for i, result in enumerate(results):
    n_nodes = result["n_nodes"]
    aiming_trace_resistance = result["trace_resistance"]
    aiming_h_trace_length = aiming_trace_resistance / h_resistance_per_length
    print(aiming_h_trace_length)
    for length in np.arange(length_min, 100, 1):
        y = sy.N(length * sy.cos((angle / 180) * sy.pi))
        z = sy.N(length * sy.sin((angle / 180) * sy.pi))

        # print(y, z)
        line_positions = np.array(
            [[0, 0, 0], [0, y / 2, z / 2], [0, y, z]], dtype=float
        )
        # line_positions = np.array([[0, 0, 0], [0, 0, length / 2], [0, 0, length]])

        trace = trace_filling(
            line_positions=line_positions,
            fill_area_radius=fill_area_radius,
            dy=1.2,
            dz=1.2,
        )

        h_trace_length = 0.0
        v_trace_length = 0.0
        for s_pos, t_pos in zip(trace[:-1], trace[1:]):
            vec = t_pos - s_pos
            mag = np.linalg.norm(vec)
            if (vec[2] > 0) or (vec[2] < 0):
                # vertical line
                v_trace_length += mag
            else:
                # horizontal line
                h_trace_length += mag
        if h_trace_length > aiming_h_trace_length:
            results[i]["cylinder_length"] = length
            length_min = length - 1
            break
    print(n_nodes, length, h_trace_length)


df1 = pd.DataFrame(results)
df1["n_wires"] = "one"
df2 = pd.DataFrame(
    data=np.array([list(df1["n_nodes"]), [double_length] * len(df1)]).T,
    columns=["n_nodes", "cylinder_length"],
)
df2["n_wires"] = "two"
df2["trace_resistance"] = aiming_trace_resistance_dbl
df = pd.concat((df1, df2))
df.to_csv("fab_scale.csv", index=False)

import pandas as pd
import matplotlib.pyplot as plt
import matplotlib
import seaborn as sns

df = pd.read_csv("fab_scale_5mm.csv")
df["trace_resistance"] /= 1000

# resistance value
matplotlib.rcParams.update({"font.size": 13})
fig, ax = plt.subplots(figsize=(4, 3))
ax.plot(
    df["n_nodes"][df["n_wires"] == "one"],
    df["trace_resistance"][df["n_wires"] == "one"],
    color="#6FAED4",
    label="Single",
)
ax.plot(
    df["n_nodes"][df["n_wires"] == "two"],
    df["trace_resistance"][df["n_wires"] == "two"],
    color="#F98A45",
    label="Double",
)
ax.set_xlabel(r"# of touchpoints")
ax.set_ylabel(r"Resistance per line (k$\Omega$)")
ax.legend(bbox_to_anchor=(0.5, 1.05))
ax.spines[["right", "top"]].set_visible(False)
# ax.yaxis.grid(True, linewidth=0.3, color="#CCCCCC")
# ax.set_axisbelow(True)
ax.set_ylim([0, 220])
fig.tight_layout()
plt.savefig("fab_scale_resistance.pdf")
plt.show()
plt.close()

# cylinder length
fig, ax = plt.subplots(figsize=(4, 3))
ax.plot(
    df["n_nodes"][df["n_wires"] == "one"],
    df["cylinder_length"][df["n_wires"] == "one"],
    color="#6FAED4",
    label="Single",
)
ax.plot(
    df["n_nodes"][df["n_wires"] == "two"],
    df["cylinder_length"][df["n_wires"] == "two"],
    color="#F98A45",
    label="Double",
)
ax.set_xlabel(r"# of touchpoints")
ax.set_ylabel(r"Conduit length (mm)")
ax.legend(bbox_to_anchor=(0.5, 1.05))
ax.spines[["right", "top"]].set_visible(False)
# ax.yaxis.grid(True, linewidth=0.3, color="#CCCCCC")
# ax.set_axisbelow(True)
ax.set_ylim([0, df["cylinder_length"].max() + 9])
fig.tight_layout()
plt.savefig("fab_scale_length.pdf")
plt.show()
plt.close()

## combine two results

import pandas as pd
import matplotlib.pyplot as plt
import matplotlib
import seaborn as sns

df_5mm = pd.read_csv("fab_scale_5mm.csv")
df_5mm["trace_resistance"] /= 1000


df_10mm = pd.read_csv("fab_scale_10mm.csv")
df_10mm["trace_resistance"] /= 1000


# cylinder length
fig, ax = plt.subplots(figsize=(4, 3))
ax.plot(
    df_5mm["n_nodes"][df_5mm["n_wires"] == "one"],
    df_5mm["cylinder_length"][df_5mm["n_wires"] == "one"],
    color="#6FAED4",
    label="Single (\u2300 5mm)",
)
ax.plot(
    df_5mm["n_nodes"][df_5mm["n_wires"] == "two"],
    df_5mm["cylinder_length"][df_5mm["n_wires"] == "two"],
    color="#F98A45",
    label="Double (\u2300 5mm)",
)

ax.plot(
    df_10mm["n_nodes"][df_10mm["n_wires"] == "one"],
    df_10mm["cylinder_length"][df_10mm["n_wires"] == "one"],
    color="#6FAED4",
    linestyle="--",
    label="Single (\u2300 10mm)",
)
ax.plot(
    df_10mm["n_nodes"][df_10mm["n_wires"] == "two"],
    df_10mm["cylinder_length"][df_10mm["n_wires"] == "two"],
    color="#F98A45",
    linestyle="--",
    label="Double (\u2300 10mm)",
)
ax.set_xlabel(r"# of touchpoints")
ax.set_ylabel(r"Conduit length (mm)")
ax.legend(bbox_to_anchor=(0.5, 1.05))
ax.spines[["right", "top"]].set_visible(False)
# ax.yaxis.grid(True, linewidth=0.3, color="#CCCCCC")
# ax.set_axisbelow(True)
ax.set_ylim([0, df_5mm["cylinder_length"].max() + 9])
fig.tight_layout()
plt.savefig("fab_scale_length.pdf")
plt.show()
plt.close()
