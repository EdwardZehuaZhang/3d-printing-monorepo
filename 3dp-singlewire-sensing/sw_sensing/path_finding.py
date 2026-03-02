"""
Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International Public License
https://creativecommons.org/licenses/by-nc-sa/4.0/legalcode

Copyright (c) 2025, Takanori Fujiwara and S. Sandra Bae
All rights reserved.
"""

import warnings

import numpy as np
import pyvista as pv

from scipy.linalg import norm
from scipy.ndimage import gaussian_filter1d


def _astar(g, source, target, weights, node_positions, h=None):
    import graph_tool.all as gt

    class Visitor(gt.AStarVisitor):

        def __init__(self, touched_v, touched_e, target):
            self.touched_v = touched_v
            self.touched_e = touched_e
            self.target = target

        def discover_vertex(self, u):
            self.touched_v[u] = True

        def examine_edge(self, e):
            self.touched_e[e] = True

        def edge_relaxed(self, e):
            if e.target() == self.target:
                raise gt.StopSearch()

    def gen_h(target, pos):
        return lambda v: np.sqrt(sum((pos[int(v)] - pos[int(target)]) ** 2))

    touch_v = g.new_vertex_property("bool")
    touch_e = g.new_edge_property("bool")
    if h is None:
        h = gen_h(target, node_positions)
    _, pred_map = gt.astar_search(
        g,
        source=source,
        weight=weights,
        visitor=Visitor(touch_v, touch_e, target),
        heuristic=h,
    )

    return gt.shortest_path(g, source, target, pred_map=pred_map)


def graph_based_path_finding(
    circuit_node_positions,
    g,
    method="djikstra",
    smoothing=True,
    smoothing_sigma=3,
    min_space=4,
    min_space_violation_penalty=100,
    graph_copy=False,
):
    import graph_tool.all as gt

    if graph_copy:
        g_ = gt.Graph(g)
    else:
        g_ = g

    g_node_positions = g_.vp["pos"].get_2d_array(range(len(g_.vp["pos"][0]))).T

    g_touch_nodes = [
        np.argsort(norm(g_node_positions - pos, axis=1))[0]
        for pos in circuit_node_positions
    ]

    set_of_path_positions = []
    for i, (s, t) in enumerate(zip(g_touch_nodes[:-1], g_touch_nodes[1:])):
        if method == "djikstra":
            path, passed_edges = gt.shortest_path(
                g_,
                source=g_.vertex(s),
                target=g_.vertex(t),
                weights=g_.edge_properties["weight"],
            )
        elif method == "astar":
            path, passed_edges = _astar(
                g_,
                source=g_.vertex(s),
                target=g_.vertex(t),
                weights=g_.edge_properties["weight"],
                node_positions=g_node_positions,
            )
        else:
            warnings.warn("selected method is not supported")
        path_positions = g_node_positions[np.array(path, dtype=int)]
        if smoothing:
            path_positions = gaussian_filter1d(
                path_positions, sigma=smoothing_sigma, mode="nearest", axis=0
            )

        min_space_ = min_space if np.ndim(min_space) == 0 else min_space[i]
        if min_space_ > 0:
            # put very large dist/weight for used and close edges
            closing_edges = passed_edges
            for path_node in path:
                path_node = g_.vertex(path_node)
                dist_map, pred_map = gt.shortest_distance(
                    g_,
                    source=path_node,
                    weights=g_.edge_properties["weight"],
                    max_dist=min_space_,
                    pred_map=True,
                )

                for related_node in np.where(dist_map.a < np.inf)[0]:
                    related_node = g_.vertex(related_node)
                    _, related_edges = gt.shortest_path(
                        g_, path_node, related_node, pred_map=pred_map
                    )
                    closing_edges += related_edges
                closing_edge_indices = [g_.edge_index[e] for e in closing_edges]
                g_.ep["weight"].a[closing_edge_indices] += min_space_violation_penalty

        set_of_path_positions.append(path_positions)

    return set_of_path_positions


def _delete_pos(positions, self_pos, threshold=1e-10):
    positions_ = []
    for pos in positions:
        if norm(pos - self_pos) >= threshold:
            positions_.append(pos)
    return positions_


def radial_ray_intersect_pos(
    contour,
    ray_start,
    non_intersect_contour,
    ray_radius=None,
    ray_radial_resolution=30,
    tolerance=1e-4,
):
    if ray_radius is None:
        # diagonal distance of bounding box
        ray_radius = (
            norm(np.array(contour.bounds[1::2]) - np.array(contour.bounds[0::2])) / 2
        )
    ray_stops = pv.Sphere(
        radius=ray_radius,
        center=ray_start,
        direction=(0.0, 0.0, 1.0),
        theta_resolution=ray_radial_resolution,
        phi_resolution=ray_radial_resolution,
    ).points

    intersect_positions = []
    for ray_stop in ray_stops:
        intersects, _ = contour.ray_trace(ray_start, ray_stop)
        if len(intersects) > 0:
            for pos in intersects:
                tol_mag = norm(pos - ray_start) * tolerance
                tmp_positions, _ = non_intersect_contour.ray_trace(ray_start, pos)
                # take out positions very close to ray_start and pos
                tmp_positions = _delete_pos(tmp_positions, ray_start, tol_mag)
                tmp_positions = _delete_pos(tmp_positions, pos, tol_mag)
                if len(tmp_positions) == 0:
                    intersect_positions.append(intersects)
    if len(intersect_positions) > 0:
        intersect_positions = np.vstack(intersect_positions)
        closest_pos = intersect_positions[
            np.argsort(norm(intersect_positions - ray_start, axis=1))[0]
        ]
    else:
        warnings.warn(
            "With current contours and rays, no intersected pos is found. Instead, ray start pos is returned."
        )
        closest_pos = ray_start

    return closest_pos


def contour_based_path_finding(
    circuit_node_positions, surface, contours, ray_radial_resolution=30, tolerance=1e-4
):
    outermost_node_positions = np.array(
        [
            radial_ray_intersect_pos(
                contour=contours[-1],
                ray_start=ray_start,
                non_intersect_contour=surface,
                ray_radial_resolution=ray_radial_resolution,
                tolerance=tolerance,
            )
            for ray_start in circuit_node_positions
        ]
    )

    paths = []
    for s_pos, t_pos in zip(
        outermost_node_positions[:-1], outermost_node_positions[1:]
    ):
        intersected = True
        s_path = [s_pos]
        t_path = [t_pos]
        s_layer = len(contours) - 1
        t_layer = len(contours) - 1
        tol_mag = norm(t_pos - s_pos) * tolerance
        while intersected and ((s_layer >= 0) and (t_layer >= 0)):
            ray_casting_layer = max(s_layer, t_layer)
            intersect_positions, _ = contours[ray_casting_layer].ray_trace(
                s_path[-1], t_path[-1]
            )
            intersect_positions = _delete_pos(intersect_positions, s_path[-1], tol_mag)
            intersect_positions = _delete_pos(intersect_positions, t_path[-1], tol_mag)

            if len(intersect_positions) == 0:
                intersected = False
            else:
                # find one close to intersected point
                s_min_dist = norm(intersect_positions - s_path[-1], axis=1).min()
                t_min_dist = norm(intersect_positions - t_path[-1], axis=1).min()

                # update the contour layer for source or target closer to intersected point
                if (s_min_dist < t_min_dist) or ((s_layer > 0) and (t_layer == 0)):
                    s_layer -= 1
                    pos = radial_ray_intersect_pos(
                        contour=contours[s_layer],
                        ray_start=s_path[-1],
                        non_intersect_contour=surface,
                        ray_radial_resolution=ray_radial_resolution,
                        tolerance=tolerance,
                    )
                    s_path.append(pos)
                elif (s_min_dist >= t_min_dist) or ((t_layer > 0) and (s_layer == 0)):
                    t_layer -= 1
                    pos = radial_ray_intersect_pos(
                        contour=contours[t_layer],
                        ray_start=t_path[-1],
                        non_intersect_contour=surface,
                        ray_radial_resolution=ray_radial_resolution,
                        tolerance=tolerance,
                    )
                    t_path.append(pos)
                else:
                    s_layer -= 1
                    t_layer -= 1

        paths.append(s_path + t_path[::-1])  # reversing t_path

    return paths
