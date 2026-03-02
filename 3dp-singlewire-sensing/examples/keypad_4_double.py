import json
import time
from pathlib import Path

import numpy as np
import pyvista as pv

import sw_sensing.geom_utils as gu
from sw_sensing.path_finding import graph_based_path_finding
from sw_sensing.geometry import (
    node_order_selection,
    generate_all_geometries,
    default_args,
)
from sw_sensing.single_wiring_optimization import optimize_resistances

if __name__ == "__main__":
    start_time = time.perf_counter()

    def log_step(message: str) -> None:
        elapsed = time.perf_counter() - start_time
        print(f"[{elapsed:7.2f}s] {message}", flush=True)

    mesh_file_name = "keypad_4"
    log_step("Loading mesh and selection data")
    models_dir = Path(__file__).resolve().parent.parent / "models"
    surface = pv.get_reader(models_dir / f"{mesh_file_name}.stl").read()
    surface = surface.scale([1, 1, 1.5])
    # surface = surface.rotate_x(45, inplace=False)
    # surface = surface.rotate_y(45, inplace=False)
    surface.save(models_dir / f"{mesh_file_name}_adjusted.stl")
    with open(models_dir / f"{mesh_file_name}_selection.json", "r") as f:
        set_of_seleceted_vertex_indices = json.load(f)

    ## 1. Node preparation
    # NOTE: when node_radius is too small. Clipping by surface tends cause excessive memory use.
    # But, Idk why this happens. Likely VTK's implementation problem
    node_radius = 4
    node_positions = gu.selected_polygons_to_node_positions(
        set_of_seleceted_vertex_indices, mesh=surface
    )
    nodes = np.arange(len(node_positions))

    connection_start_node = nodes[0]
    connection_end_node = nodes[-1]

    node_order = node_order_selection(
        initial_order=nodes,
        method="asis",
        start_node=connection_start_node,
        end_node=connection_end_node,
    )
    log_step("Node preparation is done")

    ## 2. Link path finding
    log_step("Voxelizing surface (heavy step)")
    voxels = gu.surface_to_voxels(surface, resolution=150)
    log_step("Clipping inner voxels")
    min_dist_from_surface = -3
    inner_voxels = voxels.clip_scalar(
        scalars="implicit_distance", value=min_dist_from_surface
    )
    log_step("Building voxel graph (heavy step)")
    voxel_graph = gu.pointset_to_graph(inner_voxels, n_neighbors=10)
    # inner_voxels.point_data["implicit_distance"]

    log_step("Running graph-based path finding (heavy step)")
    set_of_link_path_positions = graph_based_path_finding(
        node_positions[node_order],
        g=voxel_graph,
        min_space=10,
        min_space_violation_penalty=300,
    )
    log_step("Link path finding is done")

    from sw_sensing.geometry import prepare_link_objects
    from sensing_network.convert_utils import combine_polydata_objects

    # p = pv.Plotter()
    # p.add_mesh(surface, show_edges=False, opacity=0.1, color="green", lighting=True)
    # p.add_mesh(
    #     combine_polydata_objects(prepare_link_objects(set_of_link_path_positions)),
    #     color="white",
    #     opacity=0.5,
    # )
    # p.show()

    ## 3. Geometry preparation
    (
        combined_node_object,
        combined_link_object,
        combined_resistor_object,
    ) = generate_all_geometries(
        surface=surface,
        node_order=node_order,
        node_positions=node_positions,
        connection_start_node=connection_start_node,
        connection_end_node=connection_end_node,
        set_of_link_path_positions=set_of_link_path_positions,
        save_file_basename=str(models_dir / mesh_file_name),
        node_kwargs={
            **default_args(generate_all_geometries)["node_kwargs"],
            "clip_by_surface": True,
            "radius": node_radius,  # overwiring default param
            "surface_voxels": voxels,
            "padding_from_surface": -0.5,
            "connection_node_directions": np.array([[0, 1, 0], [0, 1, 0]]),
            "connection_node_length": 5,
        },
        link_kwargs={
            **default_args(generate_all_geometries)["link_kwargs"],
            "radius": [2.0] + [3.5] * (len(set_of_link_path_positions) - 2) + [2.0],
        },
        # resistor_trace_fill_kwargs={
        #     **default_args(generate_all_geometries)["resistor_trace_fill_kwargs"],
        #     "aiming_resistance": 2000e3,
        #     "max_fill_area_radius": [1.0]
        #     + [2.0] * (len(set_of_link_path_positions) - 2)
        #     + [1.0],
        #     # "min_dy": 0.8, # NOTE: probably we should use these instead of 0.65 and 1.0
        #     # "min_dz": 1.2,
        # },
        resistor_trace_fill_kwargs={
            **default_args(generate_all_geometries)["resistor_trace_fill_kwargs"],
            "aiming_resistance": 2000e3,
            "learning_rate": 0.1,
            "max_iterations": 300,
            "no_overlap_margin": 2.5,
            "max_fill_area_radius": [1.5]
            + [2.5] * (len(set_of_link_path_positions) - 2)
            + [1.5],
            "min_dy": 1.2,
            "min_dz": 1.2,
        },
        resistor_trace_kwargs={"horizontal_width": 0.8, "vertical_width": 1.2},
        node_to_resistor_trace_kwargs={
            "trace_horizontal_width": 0.8,
            "trace_vertical_width": 1.2,
        },
    )
    log_step("Geometry preparation is done")

    with open(models_dir / f"{mesh_file_name}_double_wire_resistance.txt", "w") as f:
        f.write("Can be anything (e.g., 1M ohm)\n")
    log_step("Output files are written")

    ## Plotting for preview
    p = pv.Plotter()
    p.add_mesh(surface, show_edges=False, opacity=0.1, color="green", lighting=True)
    for path_positions in set_of_link_path_positions:
        for s_pos, t_pos in zip(path_positions[:-1], path_positions[1:]):
            p.add_mesh(pv.Line(s_pos, t_pos), color="blue", line_width=5)
        p.add_mesh(pv.PolyData(path_positions[0]), color="blue", point_size=20)
        p.add_mesh(pv.PolyData(path_positions[-1]), color="blue", point_size=20)
    p.add_mesh(combined_node_object, color="red")
    p.add_mesh(combined_link_object, color="white", opacity=0.5)
    p.add_mesh(combined_resistor_object, color="red")
    log_step("Opening PyVista preview window")
    p.show()
