"""
Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International Public License
https://creativecommons.org/licenses/by-nc-sa/4.0/legalcode

Copyright (c) 2025, Takanori Fujiwara and S. Sandra Bae
All rights reserved.
"""

import warnings

import numpy as np
import pyvista as pv

from scipy.interpolate import interp1d
from scipy.linalg import norm
from scipy.sparse import triu
from sklearn.neighbors import kneighbors_graph
from sensing_network.convert_utils import combine_polydata_objects


def selected_polygons_to_node_positions(set_of_selected_poly_indices, mesh):
    if mesh.is_all_triangles:
        n = 3
        is_all_same_size = True
    else:
        n = mesh.faces[0]
        is_all_same_size = len(np.unique(mesh.faces[:: n + 1])) == 1

    if not is_all_same_size:
        warnings.warn(
            """Mesh with Non-same-size-polygons is not supported for now. This is due to long loading time of vertex info.Consider triangulating meshes"""
        )
    else:
        vertex_positions = mesh.points[
            np.delete(mesh.faces, np.arange(0, len(mesh.faces), n + 1))
        ]
        set_of_region_vertices = [
            np.reshape(vertex_positions[indices], (len(indices) // n, n, 3))
            for indices in set_of_selected_poly_indices
        ]
        node_positions = np.array(
            [centroid_close_to_center(vertices) for vertices in set_of_region_vertices]
        )
    return node_positions


def prepare_graph(links, link_weights, node_positions):
    import graph_tool.all as gt

    g = gt.Graph(directed=False)
    g.add_edge_list(links)

    # NOTE ep: alias of edge_properties, vp: alias of vertex_properties
    g.ep["weight"] = g.new_edge_property("float")
    g.ep["weight"].a = link_weights
    g.vp["pos"] = g.new_vertex_property("vector<float>")
    g.vp["pos"].set_2d_array(node_positions.T)

    return g


def pointset_to_graph(pointset, n_neighbors=6):
    if type(pointset) == pv.PolyData:
        # expecting surface data
        pv_edges = pointset.extract_all_edges()
        # pyvista's lines contains number of points for each line (number of points, p0, p1, p2, ...)
        pv_lines = pv_edges.lines
        pv_lines = np.delete(pv_lines, np.arange(0, len(pv_lines), 3))
        links = np.vstack([pv_lines[0::2], pv_lines[1::2]]).T
    else:
        # expecting voxels (but this part should work for other pointset)
        A = kneighbors_graph(pointset.points, n_neighbors=n_neighbors, mode="distance")
        A = A + A.T - A * A.T  # symmertrize
        A.eliminate_zeros()
        A = triu(A, k=1, format="coo")  # get only half
        A = A.tocoo()
        links = np.vstack((A.row, A.col)).T

    distances = norm(
        pointset.points[links[:, 0]] - pointset.points[links[:, 1]], axis=1
    )
    g = prepare_graph(links, link_weights=distances, node_positions=pointset.points)
    return g


def contour_data_to_contours(contour_data):
    contour_vals = np.unique(contour_data.point_data["implicit_distance"])
    contour_vals
    contour_step = 1
    if len(contour_vals):
        contour_step = contour_vals[1] - contour_vals[0]
    contour_clip_ranges = [
        [val - contour_step / 2, val + contour_step / 2] for val in contour_vals
    ]

    contours = []
    for clip_range in contour_clip_ranges:
        contour = contour_data.clip_scalar(
            scalars="implicit_distance", value=clip_range[0], invert=False
        )
        contour = contour.clip_scalar(
            scalars="implicit_distance", value=clip_range[1], invert=True
        )
        if contour.n_points > 0:
            contours.append(contour)

    return contours


def extract_contours(voxels, n_contours=6, point_data_key="implicit_distance"):
    # convert float64 to 32 to avoid capturing unneccesary small diffs
    voxels.point_data[point_data_key] = voxels.point_data[point_data_key].astype(
        "float32"
    )
    contour_data = voxels.contour(n_contours, scalars=point_data_key)
    contours = contour_data_to_contours(contour_data)

    return contours


def surface_to_voxels(surface, resolution=200, compute_implicit_distance=True):
    voxels = pv.voxelize(surface, density=surface.length / resolution)
    if compute_implicit_distance:
        voxels.compute_implicit_distance(surface, inplace=True)
    return voxels


def surface_to_contours(
    surface, n_contours=6, voxel_resolution=200, return_voxels=False
):
    voxels = surface_to_voxels(
        surface=surface, resolution=voxel_resolution, compute_implicit_distance=True
    )
    contours = extract_contours(voxels, n_contours=n_contours)

    if return_voxels:
        return contours, voxels
    else:
        return contours


def filter_contours(contours, surface, min_dist_from_surface=1):
    contours_ = []
    for contour in contours:
        if -contour.point_data["implicit_distance"][0] >= min_dist_from_surface:
            contours_.append(contour)
    return contours_


def trace_to_cylinders(
    trace, horizontal_width=0.5, vertical_width=1.0, n_sides=12, z_tolerance=1e-3
):
    cylinders = []
    for s_pos, t_pos in zip(trace[:-1], trace[1:]):
        vec = t_pos - s_pos
        mag = norm(vec)
        if np.abs(vec[2]) > z_tolerance:
            # vertical line
            radius = vertical_width / 2
            height = mag + horizontal_width
        else:
            # horizontal line
            radius = horizontal_width / 2
            height = mag + horizontal_width
        cylinders.append(
            pv.Cylinder(
                (s_pos + t_pos) / 2,
                direction=vec,
                radius=radius,
                height=height,
                resolution=n_sides,
                capping=True,
            )
        )
    return combine_polydata_objects(cylinders)


def staircasing(s_pos, t_pos, direction=[0, 0, 1], step=0.5):
    direction = np.array(direction) / norm(direction)
    vec = t_pos - s_pos
    u = vec / norm(vec)
    orth_proj_vec_mag = direction @ vec
    orth_proj_u_mag = direction @ u

    path_positions = [s_pos]
    if orth_proj_vec_mag == 0:
        path_positions.append(t_pos)
    else:
        d = 0
        if orth_proj_vec_mag < 0:
            step = -step
        for d in np.arange(step, orth_proj_vec_mag, step):
            path_positions.append(path_positions[-1] + direction * step)
            path_positions.append(
                path_positions[-1] + u * step / orth_proj_u_mag - direction * step
            )

        remains = orth_proj_vec_mag - d
        path_positions.append(path_positions[-1] + direction * remains)
        path_positions.append(
            path_positions[-1] + u * remains / orth_proj_u_mag - direction * remains
        )
    return np.vstack(path_positions)


def centroid_close_to_center(polygons, return_index=False):
    # from set of polygons, select a centroid of one polygon close to the center area
    centroids = polygons.mean(axis=1)
    center_pos = centroids.mean(axis=0)
    close_to_center_idx = np.argmin(norm(centroids - center_pos, axis=1))
    if return_index:
        return centroids[close_to_center_idx], close_to_center_idx
    else:
        return centroids[close_to_center_idx]


def rediscretized_path(positions, step=0.01):
    cum_dists = np.cumsum(norm(np.diff(positions, axis=0), axis=1))
    cum_dists = np.insert(cum_dists, 0, 0)
    if cum_dists[-1] > 0:
        cum_dists /= cum_dists[-1]
        polyline_f = interp1d(cum_dists, positions, axis=0)

        steps = np.arange(0, 1, step)
        steps = np.insert(steps, len(steps), 1)  # add 1 at last

        return polyline_f(steps)
    else:
        steps = np.arange(0, 1, step)
        return np.tile(positions[-1], (len(steps) + 1, 1))


# Ref: https://docs.pyvista.org/version/stable/examples/00-load/create-spline.html
def polyline_from_points(points):
    poly = pv.PolyData()
    poly.points = points
    the_cell = np.arange(0, len(points), dtype=int)
    the_cell = np.insert(the_cell, 0, len(points))
    poly.lines = the_cell
    return poly


def path_to_object(
    link_path_positions, radius=4.0, resolution=100, n_sides=4, capping=True
):
    points = rediscretized_path(link_path_positions, step=1 / resolution)
    polyline = polyline_from_points(points)
    polyline["scalars"] = np.arange(polyline.n_points)
    tube = polyline.tube(radius=radius, capping=capping, n_sides=n_sides)
    return tube


def clip_by_distance_from_surface(
    mesh,
    surface,
    margin=0,
    invert=False,
    fill_hole=True,
    precleaning=False,
    clip_error_thres=0.1,
):
    mesh.GlobalWarningDisplayOff()

    if precleaning:
        mesh = mesh.triangulate().clean(tolerance=1e-4).compute_normals()
        surface = surface.triangulate().clean(tolerance=1e-4).compute_normals()
    intersect, _, _ = mesh.intersection(surface, split_first=False, split_second=False)

    if len(intersect.points) > 0:
        compute_distance = False if margin == 0 else True
        mesh_ = mesh.clip_surface(
            surface, invert=invert, compute_distance=compute_distance
        )
        if margin != 0:
            mesh_ = mesh_.clip_scalar("implicit_distance", value=margin, invert=invert)
        if fill_hole:
            bounds = np.array(intersect.bounds)
            hole_size = norm(bounds[::2] - bounds[1::2])
            mesh_ = mesh_.fill_holes(hole_size)

        if mesh_.n_points < mesh.n_points * clip_error_thres:
            print("clip by surface is not applied due to violation of clip_error_thres")
        else:
            mesh = mesh_

    mesh.GlobalWarningDisplayOn()
    return mesh


def clip_by_distance_from_surfaces(
    mesh,
    surfaces,
    margin=0,
    invert=False,
    fill_hole=True,
    precleaning=False,
    clip_error_thres=0.1,
):
    mesh_ = mesh
    if precleaning:
        mesh_ = mesh.triangulate().clean(tolerance=1e-4).compute_normals()
        surfaces = [
            surface.triangulate().clean(tolerance=1e-4).compute_normals()
            for surface in surfaces
        ]
    for surface in surfaces:
        mesh_ = clip_by_distance_from_surface(
            mesh_,
            surface,
            margin=margin,
            invert=invert,
            fill_hole=fill_hole,
            precleaning=False,
            clip_error_thres=clip_error_thres,
        )
    return mesh_


def compute_normal(pos1, pos2, pos3):
    normal_vec = np.cross(
        np.array(pos2) - np.array(pos1), np.array(pos3) - np.array(pos1)
    )
    return normal_vec / norm(normal_vec)
