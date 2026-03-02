import time
import json

import numpy as np
import pandas as pd
import pyvista as pv

import sw_sensing.geom_utils as gu
from sw_sensing.path_finding import graph_based_path_finding
from sw_sensing.geometry import (
    node_order_selection,
    generate_all_geometries,
    default_args,
    prepare_node_objects,
    prepare_link_objects,
    trace_filling_with_aiming_resistance,
    prepare_resistor_trace_objects,
    traces_from_nodes_to_resistors,
    preprare_node_resistor_trace_related_objects,
    combine_polydata_objects,
)

from sw_sensing.single_wiring_optimization import optimize_resistances


def preprocess(
    mesh_file_path,
    selection_file_path,
    connection="single",
    mesh_file_path2=None,
    node_order=None,
):
    surface = pv.get_reader(mesh_file_path).read()
    with open(selection_file_path, "r") as f:
        set_of_seleceted_vertex_indices = json.load(f)
    node_positions = gu.selected_polygons_to_node_positions(
        set_of_seleceted_vertex_indices, mesh=surface
    )
    nodes = np.arange(len(node_positions))

    connection_start_node = nodes[0] if node_order is None else node_order[0]
    connection_end_node = nodes[-1] if node_order is None else node_order[-1]

    if connection == "single":
        node_positions = node_positions[:-1]
        nodes = nodes[:-1]
        connection_end_node = None

    if mesh_file_path2 is not None:
        surface = pv.get_reader(mesh_file_path2).read()

    node_order = node_order_selection(
        initial_order=nodes if node_order is None else node_order,
        method="asis",
        start_node=connection_start_node,
        end_node=connection_end_node,
    )
    print("Node prepration is done")

    return (
        surface,
        node_positions,
        node_order,
        connection_start_node,
        connection_end_node,
    )


if __name__ == "__main__":
    n_trials = 5

    samples = [
        {
            "model": "bunny",
            "connection": "single",
            "node_radius": 6,
            "min_dist_from_surface": -3,
            "voxel_resolution": 200,
            "path_min_space": 5,
            "node_padding_from_surface": -0.5,
            "link_radius": 2.5,
            "node_order": [5, 4, 3, 1, 2, 0],
        },
        {
            "model": "hilbert",
            "connection": "single",
            "node_radius": 6,
            "min_dist_from_surface": -4.5,
            "voxel_resolution": 150,
            "path_min_space": 0,
            "node_padding_from_surface": 0.0,
            "link_radius": 2.5,
        },
        {
            "model": "keypad_4",
            "connection": "single",
            "node_radius": 4,
            "min_dist_from_surface": -5,
            "voxel_resolution": 80,
            "path_min_space": 6,
            "node_padding_from_surface": -0.5,
            "link_radius": 3.0,
        },
        {
            "model": "keypad_9",
            "connection": "single",
            "node_radius": 4,
            "min_dist_from_surface": -5,
            "voxel_resolution": 140,
            "path_min_space": 6,
            "node_padding_from_surface": -0.5,
            "link_radius": 3.0,
        },
        {
            "model": "keypad_16",
            "connection": "single",
            "node_radius": 4,
            "min_dist_from_surface": -5,
            "voxel_resolution": 200,
            "path_min_space": 6,
            "node_padding_from_surface": -0.5,
            "link_radius": 3.0,
        },
        {
            "model": "land",
            "connection": "single",
            "node_radius": 6,
            "min_dist_from_surface": -3,
            "voxel_resolution": 150,
            "path_min_space": 5,
            "node_padding_from_surface": -0.1,
            "link_radius": 2.5,
            "node_order": [0, 1, 4, 2, 3, 6, 5],
        },
        {
            "model": "lion",
            "connection": "single",
            "node_radius": 7,
            "min_dist_from_surface": -5,
            "voxel_resolution": 150,
            "path_min_space": 10,
            "node_padding_from_surface": -0.5,
            "link_radius": 4.0,
            "node_order": [0, 4, 2, 5, 6, 1, 3],
            "mesh_file_path": "../../models/lion_for_selection.stl",
            "mesh_file_path2": "../../models/lion.stl",
        },
        {
            "model": "power",
            "connection": "single",
            "node_radius": 5,
            "min_dist_from_surface": -4,
            "voxel_resolution": np.array([150, 150, 400]),
            "path_min_space": 7,
            "node_padding_from_surface": 0,
            "link_radius": 3.5,
            "mesh_file_path": "../../models/power_for_selection.stl",
            "mesh_file_path2": "../../models/power.stl",
        },
    ]

    out_file = "comp_perf.csv"

    results = []
    for trial in range(n_trials):
        for sample in samples:
            # args and params
            model = sample["model"]
            connection = sample["connection"]
            print(f"{model}_{connection} (trial {trial})")
            mesh_file_path = (
                sample["mesh_file_path"]
                if "mesh_file_path" in sample
                else f"../../models/{model}.stl"
            )
            mesh_file_path2 = (
                sample["mesh_file_path2"] if "mesh_file_path2" in sample else None
            )
            selection_file_path = f"../../models/selection_{model}.json"

            min_dist_from_surface = sample["min_dist_from_surface"]
            node_radius = sample["node_radius"]
            voxel_resolution = sample["voxel_resolution"]
            path_min_space = sample["path_min_space"]
            node_padding_from_surface = sample["node_padding_from_surface"]
            link_radius = sample["link_radius"]
            node_order = sample["node_order"] if "node_order" in sample else None

            result = {
                "model": model,
                "connection": connection,
                "trial": trial,
                "x": 0.0,
                "y": 0.0,
                "z": 0.0,
                "volume": 0.0,
                "n_cells": 0,
                "n_points": 0,
                "n_voxels": 0,
                "n_vertices": 0,
                "preprocess": 0.0,
                "r_optim": 0.0,
                "voxelization": 0.0,
                "voxel_clip": 0.0,
                "graph_repr": 0.0,
                "path_finding": 0.0,
                "node_object_prep": 0.0,
                "link_object_prep": 0.0,
                "r_trace_prep": 0.0,
                "postprocess": 0.0,
            }

            # step 0-1: preprocess
            start = time.time()
            (
                surface,
                node_positions,
                node_order,
                connection_start_node,
                connection_end_node,
            ) = preprocess(
                mesh_file_path,
                selection_file_path,
                connection=connection,
                mesh_file_path2=mesh_file_path2,
                node_order=node_order,
            )
            comp_time = time.time() - start
            min_x, max_x, min_y, max_y, min_z, max_z = surface.bounds
            print("preprocess:", comp_time)
            result["preprocess"] = comp_time
            result["x"] = max_x - min_x
            result["y"] = max_y - min_y
            result["z"] = max_z - min_z
            result["n_cells"] = surface.n_cells
            result["n_points"] = surface.n_points
            result["volume"] = surface.volume

            # step 0-2: resistance optimization for single wire case
            start = time.time()
            aiming_resistance = 2000e3
            max_resistance_for_single = 450e3
            if connection == "single":
                voltage_thres = 2.5
                wire_r, trace_r, score = optimize_resistances(
                    n_nodes=len(node_positions) - 1,
                    wire_resistance_candidates=np.arange(0.2e6, 10e6 + 0.2e6, 0.2e6),
                    trace_resistance_candidates=np.arange(
                        50e3, max_resistance_for_single + 50e3, 50e3
                    ),
                )
                aiming_resistance = trace_r
            comp_time = time.time() - start
            print("r_optim:", comp_time)
            result["r_optim"] = comp_time

            # step 1: voxelization
            start = time.time()
            voxels = gu.surface_to_voxels(surface, resolution=voxel_resolution)
            comp_time = time.time() - start
            print("voxelization:", comp_time)
            result["voxelization"] = comp_time
            result["n_voxels"] = len(voxels.points)

            # step 2: voxel clipping
            start = time.time()
            inner_voxels = voxels.clip_scalar(
                scalars="implicit_distance", value=min_dist_from_surface
            )
            comp_time = time.time() - start
            print("voxel_clip:", comp_time)
            result["voxel_clip"] = comp_time
            result["n_vertices"] = len(inner_voxels.points)

            # step 3: graph representation
            start = time.time()
            voxel_graph = gu.pointset_to_graph(inner_voxels, n_neighbors=10)
            comp_time = time.time() - start
            print("graph_repr:", comp_time)
            result["graph_repr"] = comp_time

            # step 4: path finding
            start = time.time()
            set_of_link_path_positions = graph_based_path_finding(
                node_positions[node_order],
                g=voxel_graph,
                min_space=path_min_space,
                min_space_violation_penalty=300,
            )
            comp_time = time.time() - start
            print("path_finding:", comp_time)
            result["path_finding"] = comp_time

            # step 5-1: node objects preparation
            start = time.time()
            connection_nodes = [connection_start_node, connection_end_node]
            if None in connection_nodes:
                connection_nodes.remove(None)
            node_objects = prepare_node_objects(
                node_positions,
                surface=surface,
                connection_nodes=connection_nodes,
                **{
                    **default_args(generate_all_geometries)["node_kwargs"],
                    # "clip_by_surface": True,
                    "radius": node_radius,
                    "resolution": 12,
                    "boolean_intersection_tolerance": 1e-1,
                    "surface_voxels": voxels,
                    "padding_from_surface": node_padding_from_surface,
                },
            )
            comp_time = time.time() - start
            print("node_object_prep:", comp_time)
            result["node_object_prep"] = comp_time

            # step 5-2: link objects preparation
            start = time.time()
            link_objects = prepare_link_objects(
                set_of_link_path_positions,
                **{
                    **default_args(generate_all_geometries)["link_kwargs"],
                    "radius": [1.5]
                    + [link_radius] * (len(set_of_link_path_positions) - 1),
                },
            )
            comp_time = time.time() - start
            print("link_object_prep:", comp_time)
            result["link_object_prep"] = comp_time

            # step 5-3: resistor trace objects preparation
            start = time.time()
            resistor_trace_fill_kwargs = {
                **default_args(generate_all_geometries)["resistor_trace_fill_kwargs"],
                "aiming_resistance": aiming_resistance,
                "learning_rate": 0.2,
                "max_iterations": 100,
                "no_overlap_margin": min(0, node_radius + min_dist_from_surface + 1),
                "max_fill_area_radius": [1.0]
                + [link_radius - 0.5] * (len(set_of_link_path_positions) - 1),
                "min_dy": 1.2,
                "min_dz": 1.2,
            }
            resistor_traces = [
                trace_filling_with_aiming_resistance(
                    link_path_positions,
                    # no_overlap_objects=link_objects[:i] + link_objects[i + 1 :] + node_objects,
                    no_overlap_objects=node_objects,
                    **{
                        **resistor_trace_fill_kwargs,
                        "max_fill_area_radius": resistor_trace_fill_kwargs[
                            "max_fill_area_radius"
                        ][i],
                    },
                )
                for i, link_path_positions in enumerate(set_of_link_path_positions)
            ]
            resistor_trace_objects = prepare_resistor_trace_objects(
                resistor_traces, horizontal_width=0.8, vertical_width=1.2
            )
            comp_time = time.time() - start
            print("r_trace_prep:", comp_time)
            result["r_trace_prep"] = comp_time

            # step 5-4: postprocessing of objects
            start = time.time()
            node_resistor_traces = traces_from_nodes_to_resistors(
                node_order=node_order,
                node_objects=node_objects,
                resistor_traces=resistor_traces,
            )
            (
                node_resistor_trace_objects,
                node_resistor_link_objects,
            ) = preprare_node_resistor_trace_related_objects(
                node_resistor_traces,
                trace_horizontal_width=0.8,
                trace_vertical_width=1.2,
            )
            # take out connection node as it should be a filled cylinder
            connection_node_objects = [node_objects[node] for node in connection_nodes]
            connection_nodes_ = connection_nodes
            connection_nodes_.sort(reverse=True)
            for node in connection_nodes_:
                node_objects.pop(node)
            # convert links to conneciton nodes into resistor objects (filled conductive links)
            if connection_start_node is not None:
                resistor_trace_objects.append(link_objects.pop(0))
            if connection_end_node is not None:
                resistor_trace_objects.append(link_objects.pop())

            combined_node_object = combine_polydata_objects(node_objects)
            combined_link_object = combine_polydata_objects(
                link_objects + node_resistor_link_objects
            )
            combined_resistor_object = combine_polydata_objects(
                resistor_trace_objects
                + node_resistor_trace_objects
                + connection_node_objects
            )
            comp_time = time.time() - start
            print("postprocess:", comp_time)
            result["postprocess"] = comp_time

            results.append(result)
            pd.DataFrame(results).to_csv("comp_perf_volume.csv", index=False)

            # # ## Plotting for preview
            # # p = pv.Plotter()
            # # p.add_mesh(surface, show_edges=False, opacity=0.1, color="green", lighting=True)
            # # for path_positions in set_of_link_path_positions:
            # #     for s_pos, t_pos in zip(path_positions[:-1], path_positions[1:]):
            # #         p.add_mesh(pv.Line(s_pos, t_pos), color="blue", line_width=5)
            # #     p.add_mesh(pv.PolyData(path_positions[0]), color="blue", point_size=20)
            # #     p.add_mesh(pv.PolyData(path_positions[-1]), color="blue", point_size=20)
            # # p.add_mesh(combined_node_object, color="red")
            # # p.add_mesh(combined_link_object, color="white", opacity=0.5)
            # # p.add_mesh(combined_resistor_object, color="red")
            # # p.show()
