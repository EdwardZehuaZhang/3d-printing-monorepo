import pandas as pd

df = pd.read_csv("./comp_perf.csv").drop("connection", axis=1)
df_mean = df.groupby(["model"]).mean()

# big impact: voxelization, path_finding, node_object_prep, r_trace_prep,
# small impact: preprocess, r_optim, voxel_clip, graph_repr, link_object_prep, postprocess
df_mean["total"] = df_mean[
    [
        "preprocess",
        "r_optim",
        "voxelization",
        "voxel_clip",
        "graph_repr",
        "path_finding",
        "node_object_prep",
        "link_object_prep",
        "r_trace_prep",
        "postprocess",
    ]
].sum(axis=1)
df_mean["other"] = df_mean[
    [
        "preprocess",
        "r_optim",
        "voxel_clip",
        "graph_repr",
        "node_object_prep",
        "link_object_prep",
        "postprocess",
    ]
].sum(axis=1)

df_mean.to_csv("./comp_perf_mean.csv")

dimensions = []
for x, y, z in zip(
    df_mean["x"].astype(int), df_mean["y"].astype(int), df_mean["z"].astype(int)
):
    dimensions.append(f"{x}x{y}x{z}")
df_mean["dimension"] = dimensions

df_summary = df_mean[
    [
        "volume",
        "n_cells",
        "n_voxels",
        "voxelization",
        "path_finding",
        "node_object_prep",
        "r_trace_prep",
        "other",
        "total",
    ]
].reindex(
    ["bunny", "keypad_4", "keypad_9", "keypad_16", "hilbert", "land", "lion", "power"]
)
latex_table = df_summary.to_latex(
    index=True,
    # formatters={},
    float_format="{:.0f}".format,
)

print(latex_table)
