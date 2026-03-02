"""
Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International Public License
https://creativecommons.org/licenses/by-nc-sa/4.0/legalcode

Copyright (c) 2025, Takanori Fujiwara and S. Sandra Bae
All rights reserved.
"""

import inspect

import numpy as np
import pyvista as pv

from scipy.linalg import norm

# from scipy.interpolate import interp1d

from sensing_network.resistor_path_generation import ResistorPathGenerator
from sensing_network.convert_utils import combine_polydata_objects

import sw_sensing.geom_utils as gu
from sw_sensing.path_finding import graph_based_path_finding

# from sw_sensing.path_finding import contour_based_path_finding


def default_args(func):
    signature = inspect.signature(func)
    return {
        k: v.default
        for k, v in signature.parameters.items()
        if v.default is not inspect.Parameter.empty
    }


def generate_straight_path_resistor_network(
    node_positions,
    resistance=200.0e3,
    link_radius=4.0,
    node_radius=4.0,
    path_margin=0.65,
    vertical_step=1.0,
):
    nodes = np.arange(len(node_positions))
    resistor_links = [[s, t] for s, t in zip(nodes[:-1], nodes[1:])]
    resistances = [resistance] * len(resistor_links)
    in_node = nodes[-1]
    out_node = None
    resistors_h_paths = []
    resistors_v_boxes = []

    for s, t in resistor_links:
        s_pos = node_positions[s]
        t_pos = node_positions[t]

        h_paths, v_boxes, total_resist = (None, None, 0)
        adjusted_v_step = vertical_step
        while abs(resistance - total_resist) > 500 and adjusted_v_step < link_radius:
            generator = ResistorPathGenerator(
                resistance,
                node1_pos=s_pos,
                node2_pos=t_pos,
                link_radius=link_radius,
                node_radius=node_radius,
                vertical_step=adjusted_v_step,
                path_margin=path_margin,
                vertical_box_width=1.2,
                vertical_box_additional_height=0.5,
                min_node_link_overlap=0.0,
                vertical_resistivity=930.0,
                horizontal_resistivity=570.0,
            )
            h_paths_, v_boxes_, total_resist_ = generator.generate_path(
                convert_to_v_box=False
            )
            adjusted_v_step *= 1.5

            if abs(resistance - total_resist_) < abs(resistance - total_resist):
                h_paths = h_paths_
                v_boxes = v_boxes_
                total_resist = total_resist_

        if abs(resistance - total_resist) > 1000:
            print(f"Specified resistance was not able to be produced.")
            print(f"aimed: {resistance}, actual: {total_resist}")

        # adjust positions that are over the node center
        vec_nodes = t_pos - s_pos
        for i in range(len(h_paths)):
            h_path = h_paths[i]
            for j in range(len(h_path)):
                pos = np.array(h_path[j])
                vec_path_pos = pos - s_pos
                orth_vec_scale = (vec_nodes @ vec_path_pos) / (vec_nodes @ vec_nodes)
                if orth_vec_scale < 0:
                    h_paths[i][j] -= vec_nodes * orth_vec_scale
                if orth_vec_scale > 1:
                    h_paths[i][j] -= vec_nodes * (orth_vec_scale - 1)
        for i in range(len(v_boxes)):
            v_box = v_boxes[i]
            for j in range(len(v_box)):
                pos = np.array(v_box[j])
                vec_path_pos = pos - s_pos
                orth_vec_scale = (vec_nodes @ vec_path_pos) / (vec_nodes @ vec_nodes)
                if orth_vec_scale < 0:
                    v_boxes[i][j] -= vec_nodes * orth_vec_scale
                if orth_vec_scale > 1:
                    v_boxes[i][j] -= vec_nodes * (orth_vec_scale - 1)

        resistors_h_paths.append(h_paths)
        resistors_v_boxes.append(v_boxes)

    return {
        "nodes": nodes,
        "links": resistor_links,
        "node_positions": node_positions,
        "node_radius": node_radius,
        "link_radius": link_radius,
        "resistor_links": resistor_links,
        "resistances": resistances,
        "in_node": in_node,
        "out_node": out_node,
        "resistors_h_paths": resistors_h_paths,
        "resistors_v_boxes": resistors_v_boxes,
    }


def trace_filling(
    line_positions,
    fill_area_radius,
    dy=0.8,
    dz=1.0,
    no_overlap_objects=None,
    no_overlap_margin=0,
    sep_ratio_thres=0.1,
    sep_dist_thres=None,
):
    if sep_dist_thres is None:
        sep_dist_thres = dz * 10

    # separate a path into monotonically increasing x,y,z parts
    diffs = line_positions[1:] - line_positions[:-1]
    signs = (np.sign(diffs) >= 0).astype(int)
    turns = np.where(np.any(np.abs(signs[1:] - signs[:-1]) > 0, axis=1))[0] + 2
    # turns = np.where(np.abs(signs[1:] - signs[:-1])[:, 2] > 0)[0] + 2

    # separate a path when having a very small z changes
    seps = []
    for s, e in zip([0] + turns.tolist(), turns.tolist() + [len(line_positions) - 1]):
        n_positions = e - s
        dist = norm(line_positions[e] - line_positions[s])
        z_dist = np.abs(line_positions[e] - line_positions[s])[2]
        if (n_positions > 10) and ((z_dist / dist) < sep_ratio_thres):
            n_parts = int(dist / sep_dist_thres)
            if n_parts >= 2:
                seps.append(np.linspace(s, e, n_parts, endpoint=False).astype(int)[1:])

    if len(seps) > 0:
        seps = np.hstack(seps)
        seps = np.hstack((turns, seps))
        seps = np.sort(seps)
    else:
        seps = turns

    # avoid case where cylinder cannot make
    seps = seps[np.diff(seps, append=len(line_positions)) >= 2]

    sub_line_positions = [
        line_positions[s : e + 1]
        for s, e in zip([0] + seps.tolist(), seps.tolist() + [len(line_positions) - 1])
    ]
    surfaces = [
        gu.path_to_object(l_positions, radius=fill_area_radius, n_sides=12)
        .triangulate()
        .clean(tolerance=1e-4)
        .compute_normals()
        for l_positions in sub_line_positions
    ]

    horizontal_traces = []
    for i in range(len(surfaces)):
        l_positions = sub_line_positions[i]
        surface = surfaces[i]
        prev_surface = surfaces[i - 1] if i > 0 else None

        if no_overlap_objects is not None:
            surface = gu.clip_by_distance_from_surfaces(
                surface, no_overlap_objects, margin=no_overlap_margin
            )
        if surface.n_points == 0:
            continue

        if prev_surface is not None:
            surface = gu.clip_by_distance_from_surfaces(
                surface,
                [prev_surface],
                margin=np.sqrt(dy**2 + dz**2),
                # margin=1.0,
                # margin=dy**2 + dz**2,
                # margin=no_overlap_margin,
            )
        if surface.n_points == 0:
            continue

        min_x, max_x, min_y, max_y, min_z, max_z = surface.bounds
        mins = [min_x, min_y, min_z]
        maxs = [max_x, max_y, max_z]

        # scan object to draw resistor traces
        zs = np.hstack(
            (
                np.flip(np.arange(surface.center[2], min_z, -dz)),
                np.arange(surface.center[2], max_z, dz)[1:],
            )
        )

        flip_z = np.abs(l_positions[0, 2] - zs[0]) > np.abs(l_positions[0, 2] - zs[-1])
        if (
            np.abs(l_positions[0, 2] - l_positions[-1, 2])
            / norm(l_positions[0] - l_positions[-1])
        ) < sep_ratio_thres:
            if len(horizontal_traces) > 0:
                flip_z = horizontal_traces[-1][-1, 2] > l_positions[0, 2]
            else:
                flip_z = l_positions[0, 2] > l_positions[-1, 2]

        if flip_z:
            zs = np.flip(zs)

        long_dim = 0
        short_dim = 1
        if np.abs(line_positions[0, 1] - line_positions[-1, 1]) > np.abs(
            line_positions[0, 0] - line_positions[-1, 0]
        ):
            long_dim = 1
            short_dim = 0

        flip_long = (l_positions[0, long_dim] - mins[long_dim]) > (
            l_positions[0, long_dim] - maxs[long_dim]
        )
        for z in zs:
            points = []
            long_vals = np.hstack(
                (
                    np.flip(np.arange(surface.center[long_dim], mins[long_dim], -dy)),
                    np.arange(surface.center[long_dim], maxs[long_dim], dy)[long_dim:],
                )
            )
            if flip_long:
                long_vals = np.flip(long_vals)
            flip_long = not flip_long

            flip_short = (l_positions[0, short_dim] - mins[short_dim]) > (
                l_positions[0, short_dim] - maxs[short_dim]
            )

            origins = np.zeros((len(long_vals), 3))
            end_points = np.zeros((len(long_vals), 3))
            origins[:, 2] = z
            origins[:, long_dim] = long_vals
            origins[:, short_dim] = mins[short_dim]
            end_points[:, 2] = z
            end_points[:, long_dim] = long_vals
            end_points[:, short_dim] = maxs[short_dim]

            # NOTE: multi ray example1 (not fast probably due to no end points)
            # directions = end_points - origins
            # set_of_intersects, rays, _ = surface.multi_ray_trace(origins, directions)
            # for ray_id in np.unique(rays):
            #     intersects = set_of_intersects[rays == ray_id][:2]
            #     if len(intersects) == 2:
            #         if flip_short:
            #             intersects = np.flip(intersects, axis=0)
            #         flip_short = not flip_short
            #         points.append(intersects)

            # NOTE: multi ray example2 (multi ray is not precise...)
            # directions = end_points - origins
            # intersects1, rays1, _ = surface.multi_ray_trace(
            #     origins,
            #     directions,
            #     first_point=True,
            #     retry=True,
            # )
            # if len(intersects1) > 0:
            #     directions2 = end_points[rays1] - intersects1
            #     intersects2, rays2, _ = surface.multi_ray_trace(
            #         intersects1 + directions2 * 1e-2,
            #         directions2,
            #         first_point=True,
            #         retry=True,
            #     )
            #     for ray2, intersect2 in zip(rays2, intersects2):
            #         intersect1 = intersects1[ray2]
            #         intersects = np.array([intersect1, intersect2])
            #         if flip_short:
            #             intersects = np.flip(intersects, axis=0)
            #         flip_short = not flip_short
            #         points.append(intersects)

            for origin, end_point in zip(origins, end_points):
                intersects, _ = surface.ray_trace(origin, end_point)
                if len(intersects) > 0:
                    if flip_short:
                        intersects = np.flip(intersects, axis=0)
                    flip_short = not flip_short
                    points.append(intersects)

            if len(points) >= 2:
                horizontal_traces.append(np.vstack(points))

    # cut part of first and last horizontal layers to make endpoints close
    for i in range(len(horizontal_traces) - 1):
        last_pt = horizontal_traces[i][-1]
        closest_next_pt_idx = np.argmin(
            norm(last_pt - horizontal_traces[i + 1], axis=1)
        )
        horizontal_traces[i + 1] = horizontal_traces[i + 1][closest_next_pt_idx:]
        # horizontal_traces[i + 1] = horizontal_traces[i + 1][:closest_next_pt_idx + 1]

    # connect each z's layer with staircases
    staircases = [
        gu.staircasing(pts1[-1], pts2[0], step=dz / 10)[1:-1]
        for pts1, pts2 in zip(horizontal_traces[:-1], horizontal_traces[1:])
    ]
    # staircases = [
    #     gu.staircasing(
    #         pts1[-1], pts2[np.argmin(norm(pts2 - pts1[-1], axis=1))], step=dz / 10
    #     )[1:-1]
    #     for pts1, pts2 in zip(horizontal_traces[:-1], horizontal_traces[1:])
    # ]

    # # cut part of first and last horizontal layers to make endpoints close to source and target
    # dist_t0_l0 = np.min(norm(horizontal_traces[0] - line_positions[0], axis=1))
    # dist_t0_l1 = np.min(norm(horizontal_traces[0] - line_positions[-1], axis=1))
    # dist_t1_l0 = np.min(norm(horizontal_traces[-1] - line_positions[0], axis=1))
    # dist_t1_l1 = np.min(norm(horizontal_traces[-1] - line_positions[-1], axis=1))
    # if (dist_t0_l0 + dist_t1_l1) < (dist_t0_l1 + dist_t1_l0):
    #     horizontal_traces[0] = horizontal_traces[0][
    #         np.argmin(norm(horizontal_traces[0] - line_positions[0], axis=1)) :
    #     ]
    #     horizontal_traces[-1] = horizontal_traces[-1][
    #         : np.argmin(norm(horizontal_traces[-1] - line_positions[-1], axis=1))
    #     ]
    # else:
    #     horizontal_traces[0] = horizontal_traces[0][
    #         np.argmin(norm(horizontal_traces[0] - line_positions[-1], axis=1)) :
    #     ]
    #     horizontal_traces[-1] = horizontal_traces[-1][
    #         : np.argmin(norm(horizontal_traces[-1] - line_positions[0], axis=1))
    #     ]

    # aggregate all trace parts
    trace = None
    if len(horizontal_traces) > 0:
        trace = [horizontal_traces[0]]
        for staircase, h_trace in zip(staircases, horizontal_traces[1:]):
            trace.append(staircase)
            trace.append(h_trace)
        trace = np.vstack(trace)

    return trace


# def trace_filling(
#     line_positions,
#     fill_area_radius,
#     dy=0.65,
#     dz=1.0,
#     no_overlap_objects=None,
#     no_overlap_margin=0,
# ):
#     # TODO: if line_positions does not have any change in z pos
#     # this code doesn't work
#     # Also, need to add the amount of radius of a pipe to fill it completely
#     cum_dz = np.cumsum(np.abs(np.diff(line_positions[:, 2])))
#     cum_dz = np.insert(cum_dz, 0, 0)
#     polyline_f = interp1d(cum_dz, line_positions, axis=0)

#     # separate surfaces/cylinders if z coordinate sign changes
#     zs = polyline_f(cum_dz)[:, 2]
#     z_turns = np.where(np.diff(np.sign(np.diff(zs))))[0] + 2
#     # avoid case polyline_f produces one position (ie cylinder cannot make)
#     z_turns = z_turns[z_turns < len(cum_dz) - 1]
#     starts = [0] + z_turns.tolist()
#     ends = z_turns.tolist() + [len(cum_dz)]
#     surfaces = [
#         gu.path_to_object(polyline_f(cum_dz[s:e]), radius=fill_area_radius, n_sides=12)
#         .triangulate()
#         .clean(tolerance=1e-4)
#         .compute_normals()
#         for s, e in zip(starts, ends)
#     ]

#     if no_overlap_objects is not None:
#         surfaces = [
#             gu.clip_by_distance_from_surfaces(
#                 surface, no_overlap_objects, margin=no_overlap_margin
#             )
#             for surface in surfaces
#         ]

#     steps = np.arange(0, cum_dz[-1], dz)
#     steps += (cum_dz[-1] - steps[-1]) / 2  # symmetrizing from line center

#     horizontal_traces = []
#     for start, end, surface in zip(starts, ends, surfaces):
#         min_x, max_x, min_y, max_y, min_z, max_z = surface.bounds

#         # scan object to draw resistor traces
#         flip_y = line_positions[0][1] > line_positions[-1][1]
#         for step in steps[(steps >= cum_dz[start]) & (steps < cum_dz[end - 1])]:
#             points = []
#             center = polyline_f(step)
#             z = center[2]

#             ys = np.arange(min_y, max_y, dy)
#             if flip_y:
#                 ys = np.flip(ys)
#             flip_y = not flip_y

#             flip_x = line_positions[0][0] > line_positions[-1][0]
#             for y in ys:
#                 intersects, _ = surface.ray_trace([min_x, y, z], [max_x, y, z])
#                 if len(intersects) > 0:
#                     if len(intersects) > 2:
#                         # take only two points close to center and then sort by x
#                         intersects = intersects[
#                             np.argsort(norm(intersects - center, axis=1))[:2]
#                         ]
#                         intersects = intersects[np.argsort(intersects[:, 0])]
#                     if flip_x:
#                         intersects = np.flip(intersects, axis=0)
#                     flip_x = not flip_x
#                     points.append(intersects)

#             if len(points) == 0:
#                 points.append(center[None, :])
#             horizontal_traces.append(np.vstack(points))

#     # connect each z's layer with staircases
#     staircases = [
#         gu.staircasing(pts1[-1], pts2[0], step=dz / 10)[1:-1]
#         for pts1, pts2 in zip(horizontal_traces[:-1], horizontal_traces[1:])
#     ]

#     # aggregate all trace parts
#     trace = None
#     if len(horizontal_traces) > 0:
#         trace = [horizontal_traces[0]]
#         for staircase, h_trace in zip(staircases, horizontal_traces[1:]):
#             trace.append(staircase)
#             trace.append(h_trace)
#         trace = np.vstack(trace)

#     return trace


def trace_filling_with_aiming_resistance(
    line_positions,
    aiming_resistance=100e3,
    horizontal_resistivity=570.0,
    vertical_resistivity=930.0,
    max_fill_area_radius=2.0,
    min_dy=0.8,
    min_dz=1.0,
    no_overlap_objects=None,
    no_overlap_margin=0,
    sep_ratio_thres=0.1,
    sep_dist_thres=None,
    learning_rate=0.2,
    max_iterations=100,
    tol_ratio=0.05,
    verbose=False,
):
    # this produces longest possible trace
    trace = trace_filling(
        line_positions,
        fill_area_radius=max_fill_area_radius,
        dy=min_dy,
        dz=min_dz,
        no_overlap_objects=no_overlap_objects,
        no_overlap_margin=no_overlap_margin,
        sep_ratio_thres=sep_ratio_thres,
        sep_dist_thres=sep_dist_thres,
    )
    resistance = compute_resistance(
        trace,
        horizontal_resistivity=horizontal_resistivity,
        vertical_resistivity=vertical_resistivity,
    )

    line_diff = np.abs(line_positions[-1] - line_positions[0])
    adjust_z = True if line_diff[2] > line_diff[1] else False
    if resistance < aiming_resistance:
        print(f"Aimed resistance cannot achieved. Max resistance: {resistance}")
    else:
        fill_area_radius = max_fill_area_radius
        dy = min_dy
        dz = min_dz

        tol = aiming_resistance * tol_ratio
        for _ in range(max_iterations):
            if verbose:
                print(f"i-th iter: current {resistance}, aim {aiming_resistance}")
            if np.abs(aiming_resistance - resistance) < tol:
                break
            ratio = resistance / aiming_resistance
            adjusted_ratio = ratio**learning_rate
            fill_area_radius /= adjusted_ratio

            if adjust_z:
                dz *= adjusted_ratio
            else:
                dy *= adjusted_ratio

            trace = trace_filling(
                line_positions,
                fill_area_radius=fill_area_radius,
                dy=dy,
                dz=dz,
                no_overlap_objects=no_overlap_objects,
                no_overlap_margin=no_overlap_margin,
                sep_ratio_thres=sep_ratio_thres,
                sep_dist_thres=sep_dist_thres,
            )
            resistance = compute_resistance(
                trace,
                horizontal_resistivity=horizontal_resistivity,
                vertical_resistivity=vertical_resistivity,
            )

    return trace


def traces_from_nodes_to_resistors(
    node_order, node_objects, resistor_traces, connect_margin=1.0
):
    # TODO: need to handle cases where node to resistor intersect interface surface
    node_positions = [obj.center_of_mass() for obj in node_objects]
    traces = []
    # node_starts = []
    # node_ends = []
    # r_trace_starts = []
    # r_trace_ends = []
    for s, t in zip(node_order[:-1], node_order[1:]):
        s_node_pos = node_positions[s]
        t_node_pos = node_positions[t]

        r_trace = resistor_traces[np.where(np.array(node_order) == s)[0][0]]
        r_trace_start = r_trace[0]
        r_trace_end = r_trace[-1]
        node_start = s_node_pos
        node_end = t_node_pos
        if (norm(node_start - r_trace_start) + norm(node_end - r_trace_end)) > (
            norm(node_start - r_trace_end) + norm(node_end - r_trace_start)
        ):
            node_start = t_node_pos
            node_end = s_node_pos

        # slightly make two different node center positions to avoid
        # the case where electricity doesn't have to pass a node
        diff_start = r_trace_start - node_start
        diff_end = r_trace_end - node_end
        diff_start[2] = 0
        diff_end[2] = 0
        if norm(diff_start) > 0:
            diff_start /= norm(diff_start)
        if norm(diff_end) > 0:
            diff_end /= norm(diff_end)

        traces.append(
            gu.staircasing(
                node_start + diff_start * connect_margin,
                r_trace_start,
                step=(node_start - r_trace_start)[2],
            )
        )
        traces.append(
            gu.staircasing(
                node_end + diff_end * connect_margin,
                r_trace_end,
                step=(node_end - r_trace_end)[2],
            )
        )

        # node_starts.append(node_start)
        # node_ends.append(node_end)
        # r_trace_starts.append(r_trace_start)
        # r_trace_ends.append(r_trace_end)

    # traces = []
    # for node_s, node_e, r_trace_s, r_trace_e in zip(
    #     node_starts, node_ends, r_trace_starts, r_trace_ends
    # ):
    #     # slightly make two different node center positions to avoid
    #     # the case where electricity doesn't have to pass a node
    #     dist_s = norm(r_trace_s - node_s)
    #     margin_vec_s = connect_margin * np.array([1, 0, 0])
    #     if dist_s > 0:
    #         margin_vec_s = connect_margin * (r_trace_s - node_s) / dist_s

    #     dist_e = norm(r_trace_e - node_e)
    #     margin_vec_e = connect_margin * np.array([-1, 0, 0])
    #     if dist_e > 0:
    #         margin_vec_e = connect_margin * (r_trace_e - node_e) / dist_e

    #     node_s += margin_vec_s
    #     node_e += margin_vec_e

    #     traces.append(
    #         gu.staircasing(
    #             node_s,
    #             r_trace_s,
    #             step=(node_s - r_trace_s)[2],
    #         )
    #     )
    #     traces.append(
    #         gu.staircasing(
    #             node_e,
    #             r_trace_e,
    #             step=(node_e - r_trace_e)[2],
    #         )
    #     )
    return traces


# def traces_from_nodes_to_resistors(node_order, node_objects, resistor_traces):
#     node_positions = [obj.center_of_mass() for obj in node_objects]
#     traces = []
#     for s, t in zip(node_order[:-1], node_order[1:]):
#         r_trace = resistor_traces[np.where(np.array(node_order) == s)[0][0]]
#         r_trace_smaller_z_end = (
#             r_trace[0] if r_trace[0][2] < r_trace[-1][2] else r_trace[-1]
#         )
#         r_trace_larger_z_end = (
#             r_trace[0] if r_trace[0][2] > r_trace[-1][2] else r_trace[-1]
#         )
#         s_node_pos = node_positions[s]
#         t_node_pos = node_positions[t]

#         if s_node_pos[2] < t_node_pos[2]:
#             node_pos_smaller_z = s_node_pos
#             node_pos_larger_z = t_node_pos
#         elif t_node_pos[2] < s_node_pos[2]:
#             node_pos_smaller_z = t_node_pos
#             node_pos_larger_z = s_node_pos
#         else:
#             if norm(s_node_pos - r_trace[0]) < norm(t_node_pos - r_trace[0]):
#                 node_pos_smaller_z = s_node_pos
#                 node_pos_smaller_z = t_node_pos
#             else:
#                 node_pos_smaller_z = t_node_pos
#                 node_pos_larger_z = s_node_pos

#         traces.append(gu.staircasing(node_pos_smaller_z, r_trace_smaller_z_end))
#         traces.append(gu.staircasing(node_pos_larger_z, r_trace_larger_z_end))
#     return traces


def compute_resistance(trace, horizontal_resistivity, vertical_resistivity):
    resistance = 0
    for s_pos, t_pos in zip(trace[:-1], trace[1:]):
        vec = t_pos - s_pos
        mag = norm(vec)
        if (vec[2] > 0) or (vec[2] < 0):
            # vertical line
            resistivity = vertical_resistivity
        else:
            # horizontal line
            resistivity = horizontal_resistivity
        resistance += resistivity * mag
    return resistance


def gen_path_based_node_order_cost(
    node_positions,
    path_finding_fn=graph_based_path_finding,
    path_finding_args={"g": None, "min_space": 0},
    eval_fn=lambda paths: np.var(
        [norm(np.diff(positions, axis=0), axis=1).sum() for positions in paths]
    ),
):
    def cost(order):
        return eval_fn(path_finding_fn(node_positions[order], **path_finding_args))

    return cost


def node_order_selection(
    initial_order,
    method="asis",
    start_node=None,
    end_node=None,
    cost_fn=None,
    n_trials=10,
):
    order = np.array(initial_order)

    if method == "asis":
        None
    elif method == "random":
        np.random.default_rng().shuffle(order)
    elif method == "cost_fn_based":
        best_node_order = None
        best_cost = np.infty
        for i in range(n_trials):
            order = np.delete(
                order, np.where((order == start_node) | (order == end_node))[0]
            )
            if not start_node is None:
                order = np.insert(order, 0, start_node)
            if not end_node is None:
                order = np.insert(order, len(order), end_node)

            print(f"{i}-th trial")
            cost = cost_fn(order)
            print(cost, order)
            if best_cost > cost:
                best_cost = cost
                best_node_order = np.copy(order)

            np.random.shuffle(order)

        order = best_node_order

    order = np.delete(order, np.where((order == start_node) | (order == end_node))[0])
    if not start_node is None:
        order = np.insert(order, 0, start_node)
    if not end_node is None:
        order = np.insert(order, len(order), end_node)

    return order


def prepare_node_objects(
    node_positions,
    connection_nodes,
    surface,
    radius=4,
    shape="sphere",
    height=2,
    direction=[0, 0, 1],
    padding_from_surface=0,
    resolution=60,
    connection_node_length=6,
    connection_node_radius=2,
    connection_node_directions=None,
    clip_by_surface=True,
    surface_voxels=None,
    boolean_intersection_tolerance=1e-2,
):
    radii = [radius] * len(node_positions) if np.ndim(radius) == 0 else radius
    if shape == "cylinder":
        node_objects = [
            pv.Cylinder(
                center=pos,
                direction=direction,
                radius=radius,
                height=height,
                resolution=resolution,
                capping=True,
            )
            for pos, radius in zip(node_positions, radii)
        ]
    else:
        node_objects = [
            pv.Sphere(
                radius=radius,
                center=pos,
                theta_resolution=resolution,
                phi_resolution=resolution,
            )
            for pos, radius in zip(node_positions, radii)
        ]

    if clip_by_surface:
        # circuit node/touch point (this process is slow due to boolean operation between meshes)
        if surface_voxels is None:
            surface_voxels = gu.surface_to_voxels(surface, resolution=200)
        clip_surface = (
            surface_voxels.clip_scalar(
                scalars="implicit_distance", value=-padding_from_surface
            )
            .extract_surface()
            .triangulate()
        )
        for node in range(len(node_objects)):
            if not node in connection_nodes:
                node_objects[node] = node_objects[node].boolean_intersection(
                    clip_surface, tolerance=boolean_intersection_tolerance
                )

    # connection node is just a cylinder (not including a touch area)
    if connection_node_directions is None:
        connection_node_directions = np.zeros((len(connection_nodes), 3))

        n_close_points = 10
        set_of_close_points = [
            surface.points[
                np.argsort(norm(surface.points - node_positions[node], axis=1))[
                    :n_close_points
                ]
            ]
            for node in connection_nodes
        ]
        for i, points in enumerate(set_of_close_points):
            for point in points:
                connection_node_directions[i] += gu.compute_normal(
                    *surface.get_cell(surface.find_closest_cell(point)).points
                )
        connection_node_directions /= norm(connection_node_directions, axis=1)[:, None]
    for node, direction in zip(connection_nodes, connection_node_directions):
        node_objects[node] = pv.Cylinder(
            center=node_positions[node],
            direction=direction,
            radius=connection_node_radius,
            height=connection_node_length * 2,
            resolution=resolution,
            capping=True,
        )
    return node_objects


def prepare_link_objects(
    set_of_link_path_positions, radius=2.5, n_sides=12, avoid_overlaps=True
):
    radii = (
        [radius] * len(set_of_link_path_positions) if np.ndim(radius) == 0 else radius
    )
    link_objects = [
        gu.path_to_object(path_positions, radius=radius, n_sides=n_sides)
        .triangulate()
        .clean(tolerance=1e-4)
        .compute_normals()
        for path_positions, radius in zip(set_of_link_path_positions, radii)
    ]
    if avoid_overlaps:
        link_objects = [
            gu.clip_by_distance_from_surfaces(
                link_objects[i], link_objects[:i] + link_objects[i + 1 :]
            )
            .triangulate()
            .clean(tolerance=1e-4)
            .compute_normals()
            for i in range(len(link_objects))
        ]
    return link_objects


def prepare_resistor_trace_objects(
    resistor_traces, horizontal_width=0.5, vertical_width=1.0
):
    resistor_trace_objects = [
        gu.trace_to_cylinders(
            trace, horizontal_width=horizontal_width, vertical_width=vertical_width
        )
        for trace in resistor_traces
    ]
    return resistor_trace_objects


def preprare_node_resistor_trace_related_objects(
    node_resistor_traces, trace_horizontal_width=0.5, trace_vertical_width=1
):
    node_resistor_trace_objects = [
        gu.trace_to_cylinders(
            trace,
            horizontal_width=trace_horizontal_width,
            vertical_width=trace_vertical_width,
        )
        for trace in node_resistor_traces
    ]
    node_resistor_link_objects = [
        gu.trace_to_cylinders(
            trace,
            horizontal_width=trace_horizontal_width + 0.1,
            vertical_width=trace_vertical_width + 0.1,
        )
        for trace in node_resistor_traces
    ]
    return node_resistor_trace_objects, node_resistor_link_objects


def generate_all_geometries(
    surface,
    node_order,
    node_positions,
    connection_start_node,
    connection_end_node,
    set_of_link_path_positions,
    combine_objects=True,
    save_file_basename=None,
    node_kwargs={
        "clip_by_surface": True,
        "surface_voxels": None,
        "radius": 4,
        "padding_from_surface": 0.5,
        "connection_node_length": 6,
        "connection_node_radius": 2,
        "connection_node_directions": None,
    },
    link_kwargs={"radius": 2.5, "n_sides": 12, "avoid_overlaps": True},
    resistor_trace_fill_kwargs={
        "max_fill_area_radius": 2.0,
        "min_dy": 0.8,
        "min_dz": 1.0,
        "aiming_resistance": 2000e3,
        "no_overlap_margin": 2.5,
        "sep_dist_thres": None,
        "sep_ratio_thres": 0.1,
        "horizontal_resistivity": 570.0,
        "vertical_resistivity": 930.0,
        "learning_rate": 0.2,
        "max_iterations": 100,
        "tol_ratio": 0.05,
        "verbose": False,
    },
    resistor_trace_kwargs={"horizontal_width": 0.5, "vertical_width": 1.0},
    node_to_resistor_trace_kwargs={
        "trace_horizontal_width": 0.5,
        "trace_vertical_width": 1,
    },
    traces_from_nodes_to_resistors_kwargs={
        "connect_margin": 1.0,
    },
):
    connection_nodes = [connection_start_node, connection_end_node]
    if None in connection_nodes:
        connection_nodes.remove(None)

    node_objects = prepare_node_objects(
        node_positions,
        surface=surface,
        connection_nodes=connection_nodes,
        **node_kwargs,
    )
    link_objects = prepare_link_objects(set_of_link_path_positions, **link_kwargs)

    max_rad = resistor_trace_fill_kwargs["max_fill_area_radius"]
    max_fill_area_radii = (
        [max_rad] * len(set_of_link_path_positions)
        if np.ndim(max_rad) == 0
        else max_rad
    )

    aiming_r = resistor_trace_fill_kwargs["aiming_resistance"]
    aiming_resistances = (
        [aiming_r] * len(set_of_link_path_positions)
        if np.ndim(aiming_r) == 0
        else aiming_r
    )

    # TODO: trace generation between connection_start and first node
    # and connection_end and last node are redundant here (can reduce computation time)
    resistor_traces = [
        trace_filling_with_aiming_resistance(
            link_path_positions,
            # no_overlap_objects=link_objects[:i] + link_objects[i + 1 :] + node_objects,
            no_overlap_objects=node_objects,
            **{
                **resistor_trace_fill_kwargs,
                "max_fill_area_radius": max_fill_area_radii[i],
                "aiming_resistance": aiming_resistances[i],
            },
        )
        for i, link_path_positions in enumerate(set_of_link_path_positions)
    ]
    compute_resistance_kwargs = {}
    for key in ["horizontal_resistivity", "vertical_resistivity"]:
        if key in resistor_trace_fill_kwargs.keys():
            compute_resistance_kwargs[key] = resistor_trace_fill_kwargs[key]
    resistances = [
        compute_resistance(trace, **compute_resistance_kwargs)
        for trace in resistor_traces
    ]
    if connection_start_node is not None:
        resistances.pop(0)
    if connection_end_node is not None:
        resistances.pop()

    print("optimized resitances:", resistances)
    resistor_trace_objects = prepare_resistor_trace_objects(
        resistor_traces, **resistor_trace_kwargs
    )

    resistances = [
        compute_resistance(trace, horizontal_resistivity=1.0, vertical_resistivity=1.0)
        for trace in resistor_traces
    ]
    for trace in resistor_traces:
        h_trace_length = 0.0
        v_trace_length = 0.0
        for s_pos, t_pos in zip(trace[:-1], trace[1:]):
            vec = t_pos - s_pos
            mag = norm(vec)
            if (vec[2] > 0) or (vec[2] < 0):
                # vertical line
                v_trace_length += mag
            else:
                # horizontal line
                h_trace_length += mag
        print(v_trace_length / h_trace_length, v_trace_length, h_trace_length)

    node_resistor_traces = traces_from_nodes_to_resistors(
        node_order=node_order,
        node_objects=node_objects,
        resistor_traces=resistor_traces,
        **traces_from_nodes_to_resistors_kwargs,
    )
    (
        node_resistor_trace_objects,
        node_resistor_link_objects,
    ) = preprare_node_resistor_trace_related_objects(
        node_resistor_traces, **node_to_resistor_trace_kwargs
    )

    # take out connection node as it should be a filled cylinder
    connection_node_objects = [node_objects[node] for node in connection_nodes]
    connection_nodes_ = connection_nodes
    connection_nodes_.sort(reverse=True)
    for node in connection_nodes_:
        node_objects.pop(node)

    # convert links to conneciton nodes into resistor objects (filled conductive links)
    # NOTE: these are commented out to make sure the connection between a pipe and a connector node
    # if connection_start_node is not None:
    #     resistor_trace_objects.pop(0)
    # if connection_end_node is not None:
    #     resistor_trace_objects.pop()

    if connection_start_node is not None:
        resistor_trace_objects.append(link_objects.pop(0))
    if connection_end_node is not None:
        resistor_trace_objects.append(link_objects.pop())

    if combine_objects:
        combined_node_object = combine_polydata_objects(node_objects)
        combined_link_object = combine_polydata_objects(
            link_objects + node_resistor_link_objects
        )
        combined_resistor_object = combine_polydata_objects(
            resistor_trace_objects
            + node_resistor_trace_objects
            + connection_node_objects
        )

        if save_file_basename:
            combined_node_object.save(f"{save_file_basename}.node.stl")
            combined_link_object.save(f"{save_file_basename}.link.stl")
            combined_resistor_object.save(f"{save_file_basename}.resistor.stl")

        return combined_node_object, combined_link_object, combined_resistor_object
    else:
        return (
            node_objects,
            link_objects + node_resistor_link_objects,
            resistor_trace_objects
            + node_resistor_trace_objects
            + connection_node_objects,
        )


# if __name__ == "__main__":
#     mesh_file_name = "bunny_scaled"
#     surface = pv.get_reader(f"../models/{mesh_file_name}.stl").read()
#     with open("../models/selection.json", "r") as f:
#         set_of_seleceted_vertex_indices = json.load(f)

#     ##
#     ## Node preparation
#     ##
#     node_positions = gu.selected_polygons_to_node_positions(
#         set_of_seleceted_vertex_indices, mesh=surface
#     )
#     nodes = np.arange(len(node_positions))

#     connection_start_node = nodes[-1]
#     connection_end_node = nodes[-2]

#     node_order = node_order_selection(
#         initial_order=nodes,
#         method="asis",
#         start_node=connection_start_node,
#         end_node=connection_end_node,
#     )
#     # This order is good for current example selection for bunny
#     # node_order = np.array([5, 4, 3, 1, 2, 0, 6])
#     print("Node prepration is done")

#     ##
#     ## Link path finding
#     ##

#     ## A. Straight path example
#     # from sensing_network.convert_utils import output_to_stl
#     # result = generate_straight_path_resistor_network(node_positions)
#     # output_to_stl(outfile_path=f'../models/{mesh_file_name}', **result)

#     ## B. Non-straight path examples
#     # ## B-1. contour-based paths
#     # contours = gu.surface_to_contours(surface, n_contours=18)
#     # set_of_path_positions = contour_based_path_finding(node_positions[node_order],
#     #                                                    surface, contours)

#     ## B-2. Graph-based paths
#     # ## B-2-1. paths on outmost contours
#     # min_dist_from_surface = 3
#     # voxels = gu.surface_to_voxels(surface, resolution=200)
#     # inner_voxels = voxels.clip_scalar(scalars='implicit_distance',
#     #                                   value=-min_dist_from_surface)
#     # surface_graph = gu.pointset_to_graph(inner_voxels.extract_surface())
#     # set_of_path_positions = graph_based_path_finding(
#     #     node_positions[node_order], g=surface_graph)

#     ## B-2-2. paths within outmost contours (using voxel graph)
#     min_dist_from_surface = 3
#     voxels = gu.surface_to_voxels(surface, resolution=200)
#     inner_voxels = voxels.clip_scalar(
#         scalars="implicit_distance", value=-min_dist_from_surface
#     )
#     voxel_graph = gu.pointset_to_graph(inner_voxels, n_neighbors=10)
#     # NOTE: graph_based_path_finding overwrites g (if this is not good, use graph_copy=True)
#     set_of_link_path_positions = graph_based_path_finding(
#         node_positions[node_order], g=voxel_graph, min_space=5
#     )

#     print("Link path finding is done")

#     ##
#     ## Geometry preparation
#     ##
#     # There are many adjustable parameters (e.g., radius of nodes)
#     # see args in generate_all_geometries function
#     (
#         combined_node_object,
#         combined_link_object,
#         combined_resistor_object,
#     ) = generate_all_geometries(
#         surface=surface,
#         node_order=node_order,
#         node_positions=node_positions,
#         connection_start_node=connection_start_node,
#         connection_end_node=connection_end_node,
#         set_of_link_path_positions=set_of_link_path_positions,
#         save_file_basename=f"../models/{mesh_file_name}",
#         node_kwargs={
#             **default_args(generate_all_geometries)["node_kwargs"],
#             "surface_voxels": voxels,
#             "padding_from_surface": -1,
#         },
#         resistor_trace_fill_kwargs={
#             **default_args(generate_all_geometries)["resistor_trace_fill_kwargs"],
#             "aiming_resistance": 300e3,
#         },
#     )

#     print("Geometry preparation is done")

#     ##
#     ## Plotting for preview
#     ##
#     p = pv.Plotter()
#     p.add_mesh(surface, show_edges=False, opacity=0.1, color="green", lighting=True)
#     for path_positions in set_of_link_path_positions:
#         for s_pos, t_pos in zip(path_positions[:-1], path_positions[1:]):
#             p.add_mesh(pv.Line(s_pos, t_pos), color="blue", line_width=5)
#         p.add_mesh(pv.PolyData(path_positions[0]), color="blue", point_size=20)
#         p.add_mesh(pv.PolyData(path_positions[-1]), color="blue", point_size=20)
#     p.add_mesh(combined_node_object, color="red")
#     p.add_mesh(combined_link_object, color="white", opacity=0.5)
#     p.add_mesh(combined_resistor_object, color="red")
#     p.show()
