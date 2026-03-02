import json

import numpy as np
import pyvista as pv

# import pymeshfix as mf

import sw_sensing.geom_utils as gu
from sw_sensing.path_finding import (
    graph_based_path_finding,
    contour_based_path_finding,
)
from sw_sensing.geometry import (
    node_order_selection,
    generate_all_geometries,
    default_args,
)

from sensing_network.convert_utils import combine_polydata_objects

if __name__ == "__main__":
    mesh_file_name = "bunny"
    surface = pv.get_reader(f"../models/{mesh_file_name}.stl").read()
    with open(f"../models/{mesh_file_name}_selection.json", "r") as f:
        set_of_seleceted_vertex_indices = json.load(f)

    ##
    ## Node preparation
    ##
    # NOTE: when node_radius is too small. Clipping by surface tends cause excessive memory use.
    # But, Idk why this happens. Likely VTK's implementation problem
    node_radius = 6
    node_positions = gu.selected_polygons_to_node_positions(
        set_of_seleceted_vertex_indices, mesh=surface
    )
    nodes = np.arange(len(node_positions))

    connection_start_node = nodes[-2]
    connection_end_node = nodes[-1]

    node_order = node_order_selection(
        initial_order=nodes,
        method="asis",
        start_node=connection_start_node,
        end_node=connection_end_node,
    )
    node_order = np.array([5, 4, 3, 1, 2, 0, 6])
    print("Node prepration is done")

    ##
    ## Link path finding
    ##
    min_dist_from_surface = 3
    voxels = gu.surface_to_voxels(surface, resolution=200)
    inner_voxels = voxels.clip_scalar(
        scalars="implicit_distance", value=-min_dist_from_surface
    )
    voxel_graph = gu.pointset_to_graph(inner_voxels, n_neighbors=10)
    # NOTE: graph_based_path_finding overwrites g (if this is not good, use graph_copy=True)
    # reduce min_space for the out connection
    set_of_link_path_positions = graph_based_path_finding(
        node_positions[node_order],
        g=voxel_graph,
        min_space=5,
        min_space_violation_penalty=300,
        # if connection_end_node is None
        # else [5] * (len(node_order) - 2) + [5],
    )

    print("Link path finding is done")

    ##
    ## Geometry preparation
    ##
    (
        node_objects,
        link_objects,
        resistor_objects,
    ) = generate_all_geometries(
        surface=surface,
        node_order=node_order,
        node_positions=node_positions,
        connection_start_node=connection_start_node,
        connection_end_node=connection_end_node,
        set_of_link_path_positions=set_of_link_path_positions,
        save_file_basename=f"../models/{mesh_file_name}",
        combine_objects=False,
        node_kwargs={
            **default_args(generate_all_geometries)["node_kwargs"],
            "clip_by_surface": True,
            "radius": node_radius,  # overwiring default param
            "surface_voxels": voxels,
            "padding_from_surface": -0.5,
        },
        link_kwargs={
            **default_args(generate_all_geometries)["link_kwargs"],
            "radius": [1.5] + [2.5] * (len(set_of_link_path_positions) - 2) + [1.5],
        },
        resistor_trace_fill_kwargs={
            **default_args(generate_all_geometries)["resistor_trace_fill_kwargs"],
            "aiming_resistance": 2000e3,
            "learning_rate": 0.1,
            "max_iterations": 300,
            "no_overlap_margin": 2.0,
            "max_fill_area_radius": [1.0]
            + [2.0] * (len(set_of_link_path_positions) - 2)
            + [1.0],
            "min_dy": 1.2,
            "min_dz": 1.2,
            "sep_dist_thres": None,
            "sep_ratio_thres": -100,
        },
        resistor_trace_kwargs={"horizontal_width": 0.8, "vertical_width": 1.2},
        node_to_resistor_trace_kwargs={
            "trace_horizontal_width": 0.8,
            "trace_vertical_width": 1.2,
        },
    )

    print("Geometry preparation is done")

    with open(f"../models/{mesh_file_name}_double_wire_resistance.txt", "w") as f:
        f.write("Can be anything (e.g., 1M ohm)\n")

    ##
    ## Plotting for preview
    ##
    d3 = {
        "blue": "#1f77b4",
        "orange": "#ff7f0e",
        "green": "#2ca02c",
        "red": "#d62728",
        "purple": "#9467bd",
        "brown": "#8c564b",
        "pink": "#e377c2",
        "grey": "#7f7f7f",
        "yellow": "#bcbd22",
        "teal": "#17becf",
    }
    # Used in Fig. 2 (principle)
    p = pv.Plotter()
    p.add_mesh(surface, show_edges=False, opacity=0.3, color="white", lighting=True)
    for node_object, color in zip(
        node_objects, [d3["yellow"], d3["red"], d3["purple"], d3["green"], d3["orange"]]
    ):
        p.add_mesh(node_object, color=color)
    p.add_mesh(combine_polydata_objects(resistor_objects), color="#507AA6")
    p.show()

    np.array(
        [950078.5894168699, 1569330.9303017834, 816386.2246107062, 674040.7262393212]
    ) * 0.3

    # Set of figures used for the paper writing
    #
    # Fig. 3 (pipeline)
    #
    camera_position = [
        (-55.088284097053005, 125.84526043907171, 89.7252193635164),
        (182.689312705153, 102.3836738702285, 50.18428137129228),
        (0.1637080540392612, -0.003458297708279351, 0.9865027689873148),
    ]

    window_size = [1536, 1536]

    # Fig. 3-1: interface shape
    p = pv.Plotter()
    p.camera_position = camera_position
    p.add_mesh(surface, show_edges=False, opacity=1, color="white", lighting=True)
    p.show(window_size=window_size)

    # Fig. 3-2: interface shape + touch point coordinates
    p = pv.Plotter()
    p.camera_position = camera_position
    p.add_mesh(surface, show_edges=False, opacity=1, color="white", lighting=True)
    for pos in node_positions:
        sphere = pv.Sphere(radius=3, center=pos)
        p.add_mesh(sphere, show_edges=False, opacity=1, color=d3["red"], lighting=True)
    p.show(window_size=window_size)

    # Fig. 3-3: circuit skeleton
    p = pv.Plotter()
    p.camera_position = camera_position
    p.add_mesh(surface, show_edges=False, opacity=0.3, color="white", lighting=True)
    for node_object in node_objects:
        p.add_mesh(node_object, color=d3["brown"])
    for i, link_object in enumerate(link_objects[0:4]):
        p.add_mesh(
            link_object, show_edges=False, opacity=1, color=d3["yellow"], lighting=True
        )
    for i, link_object in enumerate(link_objects[6:]):
        p.add_mesh(
            link_object, show_edges=False, opacity=1, color=d3["brown"], lighting=True
        )
    for idx in [-1, -2, len(node_order) - 1, len(node_order)]:
        p.add_mesh(
            resistor_objects[idx],
            show_edges=False,
            opacity=1,
            color=d3["brown"],
            lighting=True,
        )
    p.show(window_size=window_size)

    # Fig. 3-4: fabrication data
    p = pv.Plotter()
    p.camera_position = camera_position
    p.add_mesh(surface, show_edges=False, opacity=0.3, color="white", lighting=True)
    for node_object in node_objects:
        p.add_mesh(node_object, color=d3["blue"])
    for i, link_object in enumerate(link_objects):
        p.add_mesh(
            link_object, show_edges=False, opacity=0.1, color="white", lighting=True
        )
    p.add_mesh(
        combine_polydata_objects(resistor_objects),
        show_edges=False,
        opacity=1,
        color=d3["blue"],
        lighting=True,
    )
    p.show(window_size=window_size)

    #
    # Fig. 5 (Circuit design)
    #
    camera_position = [
        (73.65604427197398, 318.61463334362287, 67.13117202763345),
        (179.27194810224816, 100.91112786549857, 57.00289856654082),
        (0.00011985884308975472, -0.04641495583765088, 0.9989222379687254),
    ]

    # Fig. 5-1: circuit skeleton
    p = pv.Plotter()
    p.camera_position = camera_position
    p.add_mesh(surface, show_edges=False, opacity=0.3, color="white", lighting=True)
    for node_object in node_objects:
        p.add_mesh(node_object, color=d3["brown"])
    for i, link_object in enumerate(link_objects[0:4]):
        p.add_mesh(
            link_object, show_edges=False, opacity=1, color=d3["yellow"], lighting=True
        )
    for i, link_object in enumerate(link_objects[6:]):
        p.add_mesh(
            link_object, show_edges=False, opacity=1, color=d3["brown"], lighting=True
        )
    for idx in [-1, -2, len(node_order) - 1, len(node_order)]:
        p.add_mesh(
            resistor_objects[idx],
            show_edges=False,
            opacity=1,
            color=d3["brown"],
            lighting=True,
        )
    p.show(window_size=window_size)

    # Fig. 5-2: circuit embedding (zoomed link)
    from sw_sensing.geometry import (
        trace_filling,
        prepare_resistor_trace_objects,
    )

    trace = trace_filling(
        set_of_link_path_positions[3], fill_area_radius=2.5, dy=0.65, dz=1.2
    )
    trace_object = prepare_resistor_trace_objects(
        [trace[544:1300]], horizontal_width=0.3, vertical_width=0.6
    )[0]
    trace_object2 = prepare_resistor_trace_objects(
        [trace[200:544]], horizontal_width=0.3, vertical_width=0.6
    )[0]
    plane = (
        pv.Plane(
            center=(120.0, 100.0, 82.35),  # 81.35),
            direction=(0.0, 0.0, 1.0),
            i_size=50,
            j_size=50,
        )
        .triangulate()
        .subdivide(3)
    )
    clipped_plane = plane.clip_surface(link_objects[2], compute_distance=True)
    clipped_plane_outline = clipped_plane.clip_scalar(
        scalars="implicit_distance", invert=False, value=-0.1
    )

    # Fig. 5-3: circuit embedding (serpentine pattern)
    p = pv.Plotter()
    light = pv.Light(color=d3["blue"], light_type="headlight", intensity=0.3)
    p.add_light(light)
    p.camera_position = [
        (110.23929959430151, 137.5133978832613, 87.67421037587593),
        (156.91382500502436, 76.44861632027659, 80.79871626644186),
        (0.175294919239674, 0.023166342889200692, 0.9842433702321274),
    ]
    p.add_mesh(
        link_objects[2],
        show_edges=False,
        opacity=0.2,
        color="#bcbd22",
        lighting=False,
    )
    p.add_mesh(trace_object, color=d3["blue"])
    p.add_mesh(trace_object2, color="white", opacity=0.1)
    p.show(window_size=window_size)

    p = pv.Plotter()
    trace_object3 = prepare_resistor_trace_objects(
        [trace[555:620]], horizontal_width=0.3, vertical_width=0.6
    )[0]
    p.camera_position = [
        (141.4645492790142, 94.34976322869292, 106.45697871620384),
        (140.05057930126944, 94.50067942733723, 70.48626518000309),
        (0.9845042068748264, -0.17087360840799332, -0.03941670452233596),
    ]
    p.add_mesh(trace_object3, color=d3["blue"])
    p.add_mesh(
        clipped_plane,
        color="black",
        lighting=False,
    )
    p.show(window_size=window_size)

    # Fig. 5-4: fabrication data
    p = pv.Plotter()
    p.camera_position = camera_position
    p.add_mesh(surface, show_edges=False, opacity=0.3, color="white", lighting=True)
    for node_object in node_objects:
        p.add_mesh(node_object, color=d3["blue"])
    for i, link_object in enumerate(link_objects):
        p.add_mesh(
            link_object, show_edges=False, opacity=0.1, color="white", lighting=True
        )
    p.add_mesh(
        combine_polydata_objects(resistor_objects),
        show_edges=False,
        opacity=1,
        color=d3["blue"],
        lighting=True,
    )
    p.show(window_size=window_size)

    #
    # Fig. 6 (path finding)
    #
    camera_position = [
        (184.20157635864365, -183.7779953517084, 113.33329584724717),
        (182.6247487324986, 101.83472642629233, 47.79359944976338),
        (0.026312405472397726, 0.22371807579408254, 0.9742986605149621),
    ]
    window_size = [1536, 1536]

    # Fig. 6-1: interface shape
    p = pv.Plotter()
    p.camera_position = camera_position
    p.add_mesh(surface, show_edges=False, opacity=1, color="white", lighting=True)
    p.show(window_size=window_size)

    # Fig. 6-2: voxel representation
    low_reso_voxels = gu.surface_to_voxels(surface, resolution=50)
    p = pv.Plotter()
    p.camera_position = camera_position
    p.add_mesh(
        low_reso_voxels, show_edges=True, opacity=1, color="white", lighting=True
    )
    p.show(window_size=window_size)

    # Fig. 6-3: inner voxel representation
    low_resp_inner_voxels = low_reso_voxels.clip_scalar(
        scalars="implicit_distance", value=-2.8
    )
    p = pv.Plotter()
    p.camera_position = camera_position
    p.add_mesh(
        low_resp_inner_voxels,
        show_edges=True,
        opacity=1,
        color="white",
        lighting=True,
    )
    p.show(window_size=window_size)

    # Fig. 6-4&5: graph representation
    g = gu.pointset_to_graph(low_resp_inner_voxels, n_neighbors=7)
    vertex_positions = g.vp["pos"].get_2d_array(range(len(g.vp["pos"][0]))).T
    edge_positions = vertex_positions[g.get_edges()]
    g_touch_nodes = [
        np.argsort(np.linalg.norm(vertex_positions - pos, axis=1))[0]
        for pos in node_positions
    ]

    edge_objects = [pv.Line(p[0], p[1]) for p in edge_positions]
    combined_edge_object = combine_polydata_objects(edge_objects)
    p = pv.Plotter()
    p.camera_position = camera_position
    p.add_mesh(combined_edge_object.tube(radius=0.03), color="#d0d0d0")
    # edge_positions = edge_positions.reshape(
    #     (edge_positions.shape[0] * edge_positions.shape[1], 3)
    # )
    # p.add_lines(edge_positions, color="#aaaaaa", width=0.01)
    p.add_points(
        vertex_positions,
        render_points_as_spheres=True,
        point_size=15.0,
        color=d3["teal"],
    )
    p.add_points(
        vertex_positions[g_touch_nodes],
        render_points_as_spheres=True,
        point_size=50.0,
        color=d3["red"],
    )
    p.show(window_size=window_size)

    # Fig. 6-6: path
    from sw_sensing.geometry import prepare_link_objects

    low_reso_set_of_link_path_positions = graph_based_path_finding(
        node_positions[node_order],
        g=g,
        min_space=5,
        smoothing=False,
    )
    path_objects = prepare_link_objects(low_reso_set_of_link_path_positions, radius=1)
    combined_path_object = combine_polydata_objects(path_objects)
    p = pv.Plotter()
    p.camera_position = camera_position
    p.add_mesh(combined_edge_object.tube(radius=0.01), color="#aaaaaa", opacity=0.3)
    p.add_points(
        vertex_positions,
        render_points_as_spheres=True,
        point_size=10.0,
        color=d3["teal"],
        opacity=0.4,
    )
    p.add_mesh(combined_path_object, color=d3["blue"])
    p.add_points(
        vertex_positions[g_touch_nodes],
        render_points_as_spheres=True,
        point_size=60.0,
        color=d3["red"],
    )
    p.show(window_size=window_size)
