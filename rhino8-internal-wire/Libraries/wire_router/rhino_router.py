from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Set, Tuple

import Rhino
import Rhino.DocObjects as rd
import Rhino.Geometry as rg
import Rhino.Input as ri
import Rhino.Input.Custom as ric
import System
import System.Drawing
import scriptcontext as sc

from .core import (
    GridIndex,
    GridSpec,
    NodeOrderMetrics,
    RoutingError,
    dilate_cells,
    evaluate_node_order,
    index_to_point,
    optimize_node_order_for_maximum_spacing,
    optimize_node_order_for_path,
    optimize_node_order_for_target_leg_length,
    route_node_sequence,
)


MAX_ESTIMATED_CELLS = 180000
DEFAULT_WIRE_DIAMETER_MM = 0.5
MIN_WIRE_DIAMETER_MM = 0.5
CASING_THICKNESS_MM = 0.5
BOUNDARY_CLEARANCE_MM = 0.5
PATH_SEPARATION_MM = 0.5
DEFAULT_TOUCH_NODE_FLUSH_DIAMETER_MM = 3.0
MIN_TOUCH_NODE_FLUSH_DIAMETER_MM = 0.5
TERMINAL_DIAMETER_MM = 3.0
TERMINAL_LENGTH_MM = 6.0
MAX_DP_NODES = 10
DEFAULT_MIN_TOUCH_READING_DELTA_KOHM = 10.0
MIN_ALLOWED_TOUCH_READING_DELTA_KOHM = 1.0
ORDER_MODE_THRESHOLD_FIRST = "threshold-first"
ORDER_MODE_SHORTEST_PATH_FIRST = "shortest-path-first"
ORDER_MODE_MAXIMIZE_SEPARATION = "maximize-separation"
SUGGESTED_SERIES_RESISTOR_RANGE_OHM = (470000.0, 2200000.0)
PROTO_PASTA_BASE_DIAMETER_MM = 1.75
PROTO_PASTA_RESISTANCE_KOHM_PER_100MM = (2.0, 3.5)


@dataclass(frozen=True)
class TouchNodePlacement:
    label: str
    surface_point: rg.Point3d
    center_point: rg.Point3d
    anchor_point: rg.Point3d
    outward_direction: rg.Vector3d


@dataclass(frozen=True)
class TerminalPlacement:
    label: str
    surface_point: rg.Point3d
    anchor_point: rg.Point3d
    outward_direction: rg.Vector3d
    protrudes: bool


@dataclass(frozen=True)
class TouchNodeOrderDecision:
    ordered_nodes: Tuple[TouchNodePlacement, ...]
    selected_strategy: str
    path_nodes: Tuple[TouchNodePlacement, ...]
    threshold_nodes: Tuple[TouchNodePlacement, ...]
    max_spacing_nodes: Tuple[TouchNodePlacement, ...]
    path_metrics: NodeOrderMetrics
    threshold_metrics: NodeOrderMetrics
    max_spacing_metrics: NodeOrderMetrics


def _active_doc() -> Rhino.RhinoDoc:
    if "__rhino_doc__" in globals() and __rhino_doc__ is not None:
        return __rhino_doc__
    return sc.doc


def _mm_to_model(doc: Rhino.RhinoDoc, value_mm: float) -> float:
    return value_mm * Rhino.RhinoMath.UnitScale(
        Rhino.UnitSystem.Millimeters, doc.ModelUnitSystem
    )


def _model_to_mm(doc: Rhino.RhinoDoc, value: float) -> float:
    return value * Rhino.RhinoMath.UnitScale(doc.ModelUnitSystem, Rhino.UnitSystem.Millimeters)


def _mesh_from_objref(objref: Rhino.DocObjects.ObjRef, tolerance: float) -> Optional[rg.Mesh]:
    geometry = objref.Geometry()
    if geometry is None:
        return None

    mesh: Optional[rg.Mesh] = None
    if isinstance(geometry, rg.Mesh):
        mesh = geometry.DuplicateMesh()
    elif isinstance(geometry, rg.SubD):
        mesh = rg.Mesh.CreateFromSubD(geometry, 2)
    else:
        brep = geometry if isinstance(geometry, rg.Brep) else rg.Brep.TryConvertBrep(geometry)
        if brep is None or not brep.IsSolid:
            return None

        meshing = rg.MeshingParameters.Smooth
        meshing.MinimumEdgeLength = tolerance
        meshes = rg.Mesh.CreateFromBrep(brep, meshing)
        if not meshes:
            return None
        mesh = rg.Mesh()
        for part in meshes:
            mesh.Append(part)

    if mesh is None:
        return None

    mesh.Compact()
    mesh.UnifyNormals()
    mesh.Normals.ComputeNormals()
    if not mesh.IsClosed and not mesh.IsSolid:
        return None
    return mesh


def _host_brep_from_objref(objref: Rhino.DocObjects.ObjRef, tolerance: float) -> Optional[rg.Brep]:
    geometry = objref.Geometry()
    if geometry is None:
        return None

    if isinstance(geometry, rg.Brep):
        brep = geometry.DuplicateBrep()
    elif isinstance(geometry, rg.Extrusion):
        brep = geometry.ToBrep(True)
    elif isinstance(geometry, rg.Mesh):
        brep = rg.Brep.CreateFromMesh(geometry, True)
    elif isinstance(geometry, rg.SubD):
        mesh = rg.Mesh.CreateFromSubD(geometry, 2)
        brep = rg.Brep.CreateFromMesh(mesh, True) if mesh is not None else None
    else:
        brep = rg.Brep.TryConvertBrep(geometry)

    if brep is None or not brep.IsSolid:
        return None

    brep.MergeCoplanarFaces(tolerance)
    return brep


def _select_target_geometry() -> Optional[Rhino.DocObjects.ObjRef]:
    go = ric.GetObject()
    go.SetCommandPrompt("Select one closed object to route inside")
    go.GeometryFilter = (
        rd.ObjectType.Mesh
        | rd.ObjectType.Brep
        | rd.ObjectType.Extrusion
        | rd.ObjectType.SubD
    )
    go.SubObjectSelect = False
    go.GroupSelect = False
    result = go.Get()
    if result != ri.GetResult.Object:
        return None
    return go.Object(0)


def _estimate_grid_cells(bbox: rg.BoundingBox, step: float) -> int:
    x_count = max(1, int(math.ceil((bbox.Max.X - bbox.Min.X) / step)))
    y_count = max(1, int(math.ceil((bbox.Max.Y - bbox.Min.Y) / step)))
    z_count = max(1, int(math.ceil((bbox.Max.Z - bbox.Min.Z) / step)))
    return x_count * y_count * z_count


def _auto_step(mesh: rg.Mesh, doc: Rhino.RhinoDoc, wire_diameter_mm: float) -> float:
    bbox = mesh.GetBoundingBox(True)
    diagonal = bbox.Diagonal.Length
    minimum = max(_mm_to_model(doc, wire_diameter_mm), doc.ModelAbsoluteTolerance * 2.0)
    step = max(minimum, diagonal / 100.0)

    estimated = _estimate_grid_cells(bbox, step)
    if estimated > MAX_ESTIMATED_CELLS:
        step *= (estimated / float(MAX_ESTIMATED_CELLS)) ** (1.0 / 3.0)
    return step


def _mesh_distance(mesh: rg.Mesh, point: rg.Point3d) -> float:
    mesh_point = mesh.ClosestMeshPoint(point, 0.0)
    if mesh_point is not None:
        return point.DistanceTo(mesh.PointAt(mesh_point))
    return point.DistanceTo(mesh.ClosestPoint(point))


def _mesh_bbox_center(mesh: rg.Mesh) -> rg.Point3d:
    return mesh.GetBoundingBox(True).Center


def _candidate_inward_directions(mesh: rg.Mesh, surface_point: rg.Point3d) -> List[rg.Vector3d]:
    directions: List[rg.Vector3d] = []
    mesh_point = mesh.ClosestMeshPoint(surface_point, 0.0)
    if mesh_point is not None:
        normal = mesh.NormalAt(mesh_point)
        if normal.Unitize():
            directions.append(rg.Vector3d(normal))
            directions.append(rg.Vector3d(-normal))

    bbox_center = _mesh_bbox_center(mesh)
    bbox_vector = bbox_center - surface_point
    if bbox_vector.Unitize():
        directions.append(bbox_vector)

    closest_mesh_point = mesh.ClosestPoint(bbox_center)
    center_to_mesh = closest_mesh_point - surface_point
    if center_to_mesh.Unitize():
        directions.append(center_to_mesh)

    return directions


def _inward_center_for_surface_point(
    mesh: rg.Mesh,
    surface_point: rg.Point3d,
    inset: float,
    minimum_clearance: float,
    tolerance: float,
) -> Optional[Tuple[rg.Point3d, rg.Vector3d]]:
    candidates: List[Tuple[float, rg.Point3d, rg.Vector3d]] = []
    probe_distance = max(tolerance * 4.0, inset * 0.25)
    for direction in _candidate_inward_directions(mesh, surface_point):
        inward = rg.Vector3d(direction)
        if not inward.Unitize():
            continue

        probe = surface_point + (inward * probe_distance)
        if not mesh.IsPointInside(probe, tolerance, False):
            continue

        center = surface_point + (inward * inset)
        if not mesh.IsPointInside(center, tolerance, False):
            continue

        clearance = _mesh_distance(mesh, center)
        if clearance + tolerance < minimum_clearance:
            continue

        outward = -inward
        candidates.append((clearance, center, outward))

    if not candidates:
        return None

    _, center, outward = max(candidates, key=lambda item: item[0])
    outward.Unitize()
    return center, outward


def _create_touch_node(
    mesh: rg.Mesh,
    label: str,
    surface_point: rg.Point3d,
    node_radius: float,
    tolerance: float,
) -> Optional[TouchNodePlacement]:
    anchor_offset = max(node_radius * 0.5, tolerance * 4.0)
    result = _inward_center_for_surface_point(
        mesh,
        surface_point,
        anchor_offset,
        0.0,
        tolerance,
    )
    if result is None:
        return None
    anchor_point, outward = result
    return TouchNodePlacement(
        label=label,
        surface_point=rg.Point3d(surface_point),
        center_point=rg.Point3d(surface_point),
        anchor_point=anchor_point,
        outward_direction=outward,
    )


def _create_terminal(
    mesh: rg.Mesh,
    label: str,
    surface_point: rg.Point3d,
    protrudes: bool,
    anchor_depth: float,
    minimum_clearance: float,
    tolerance: float,
) -> Optional[TerminalPlacement]:
    result = _inward_center_for_surface_point(
        mesh,
        surface_point,
        anchor_depth,
        minimum_clearance,
        tolerance,
    )
    if result is None:
        return None
    anchor_point, outward = result
    return TerminalPlacement(
        label=label,
        surface_point=rg.Point3d(surface_point),
        anchor_point=anchor_point,
        outward_direction=outward,
        protrudes=protrudes,
    )


def _orthonormal_frame(direction: rg.Vector3d) -> Tuple[rg.Vector3d, rg.Vector3d]:
    direction = rg.Vector3d(direction)
    direction.Unitize()
    reference = rg.Vector3d(0.0, 0.0, 1.0)
    if abs(direction * reference) > 0.95:
        reference = rg.Vector3d(1.0, 0.0, 0.0)
    axis_x = rg.Vector3d.CrossProduct(direction, reference)
    axis_x.Unitize()
    axis_y = rg.Vector3d.CrossProduct(direction, axis_x)
    axis_y.Unitize()
    return axis_x, axis_y


def _make_cylinder_brep(
    base_center: rg.Point3d,
    axis_direction: rg.Vector3d,
    length: float,
    radius: float,
) -> Optional[rg.Brep]:
    direction = rg.Vector3d(axis_direction)
    if not direction.Unitize():
        return None
    plane = rg.Plane(base_center, direction)
    circle = rg.Circle(plane, radius)
    cylinder = rg.Cylinder(circle, length)
    return cylinder.ToBrep(True, True)


def _brep_volume(brep: rg.Brep) -> float:
    props = rg.VolumeMassProperties.Compute(brep)
    if props is None:
        return 0.0
    return props.Volume


def _boolean_intersection(
    minuend: Sequence[rg.Brep],
    cutters: Sequence[rg.Brep],
    tolerance: float,
) -> List[rg.Brep]:
    valid_minuend = [brep for brep in minuend if brep is not None and brep.IsValid]
    valid_cutters = [brep for brep in cutters if brep is not None and brep.IsValid]
    if not valid_minuend or not valid_cutters:
        return []

    result = rg.Brep.CreateBooleanIntersection(valid_minuend, valid_cutters, tolerance)
    return list(result) if result else []


def _boolean_union(breps: Sequence[rg.Brep], tolerance: float) -> List[rg.Brep]:
    valid = [brep for brep in breps if brep is not None and brep.IsValid]
    if not valid:
        return []
    if len(valid) == 1:
        return valid

    union = rg.Brep.CreateBooleanUnion(valid, tolerance)
    return list(union) if union else valid


def _boolean_difference(
    minuend: Sequence[rg.Brep],
    subtractors: Sequence[rg.Brep],
    tolerance: float,
) -> List[rg.Brep]:
    valid_minuend = [brep for brep in minuend if brep is not None and brep.IsValid]
    valid_subtractors = [brep for brep in subtractors if brep is not None and brep.IsValid]
    if not valid_minuend:
        return []
    if not valid_subtractors:
        return valid_minuend

    difference = rg.Brep.CreateBooleanDifference(valid_minuend, valid_subtractors, tolerance)
    return list(difference) if difference else valid_minuend


def _terminal_internal_brep(
    terminal: TerminalPlacement,
    terminal_radius: float,
    terminal_length: float,
) -> Optional[rg.Brep]:
    inward = -terminal.outward_direction
    if terminal.protrudes:
        return _make_cylinder_brep(
            terminal.surface_point,
            inward,
            terminal_radius,
            terminal_radius,
        )
    return _make_cylinder_brep(
        terminal.surface_point,
        inward,
        terminal_length,
        terminal_radius,
    )


def _terminal_conductive_breps(
    terminal: TerminalPlacement,
    host_brep: rg.Brep,
    terminal_radius: float,
    terminal_length: float,
    tolerance: float,
) -> Optional[List[rg.Brep]]:
    internal = _terminal_internal_brep(terminal, terminal_radius, terminal_length)
    if internal is None:
        return None

    trimmed_internal = _boolean_intersection([internal], [host_brep], tolerance)
    if not trimmed_internal:
        return None

    original_volume = _brep_volume(internal)
    retained_volume = sum(_brep_volume(brep) for brep in trimmed_internal)
    if original_volume > tolerance and retained_volume < (original_volume * 0.9):
        return None

    if not terminal.protrudes:
        return trimmed_internal

    external = _make_cylinder_brep(
        terminal.surface_point,
        terminal.outward_direction,
        terminal_length,
        terminal_radius,
    )
    if external is None:
        return None
    return trimmed_internal + [external]


def _validate_touch_node(
    candidate: Optional[TouchNodePlacement],
    touch_nodes: Sequence[TouchNodePlacement],
    terminals: Sequence[TerminalPlacement],
    node_radius: float,
    terminal_radius: float,
    min_gap: float,
    tolerance: float,
) -> Optional[str]:
    if candidate is None:
        return "That conductive pathway node sphere diameter is too big for the selected surface location."

    for node in touch_nodes:
        edge_gap = candidate.center_point.DistanceTo(node.center_point) - (node_radius * 2.0)
        if edge_gap + tolerance < min_gap:
            return "That conductive pathway node sphere diameter is too big and overlaps another conductive pathway node."

    for terminal in terminals:
        edge_gap = candidate.center_point.DistanceTo(terminal.surface_point) - (node_radius + terminal_radius)
        if edge_gap + tolerance < min_gap:
            return "That conductive pathway node sphere diameter is too big and overlaps a terminal connector."
    return None


class _TouchNodeGetter(ric.GetPoint):
    def __init__(
        self,
        mesh: rg.Mesh,
        touch_nodes: Sequence[TouchNodePlacement],
        terminals: Sequence[TerminalPlacement],
        node_radius: float,
        terminal_radius: float,
        min_gap: float,
        tolerance: float,
    ) -> None:
        super(_TouchNodeGetter, self).__init__()
        self._mesh = mesh
        self._touch_nodes = list(touch_nodes)
        self._terminals = list(terminals)
        self._node_radius = node_radius
        self._terminal_radius = terminal_radius
        self._min_gap = min_gap
        self._tolerance = tolerance

    def OnDynamicDraw(self, e: ri.GetPointDrawEventArgs) -> None:
        super(_TouchNodeGetter, self).OnDynamicDraw(e)

        for node in self._touch_nodes:
            e.Display.DrawSphere(rg.Sphere(node.center_point, self._node_radius), System.Drawing.Color.DeepSkyBlue)

        for terminal in self._terminals:
            e.Display.DrawPoint(terminal.surface_point, Rhino.Display.PointStyle.ControlPoint, 6, System.Drawing.Color.Gold)

        preview = _create_touch_node(
            self._mesh,
            "Preview",
            e.CurrentPoint,
            self._node_radius,
            self._tolerance,
        )
        error = _validate_touch_node(
            preview,
            self._touch_nodes,
            self._terminals,
            self._node_radius,
            self._terminal_radius,
            self._min_gap,
            self._tolerance,
        )
        if preview is None:
            return

        color = System.Drawing.Color.OrangeRed if error is not None else System.Drawing.Color.LimeGreen
        e.Display.DrawSphere(rg.Sphere(preview.center_point, self._node_radius), color)


class _TerminalGetter(ric.GetPoint):
    def __init__(
        self,
        mesh: rg.Mesh,
        label: str,
        existing_terminals: Sequence[TerminalPlacement],
        terminal_radius: float,
        minimum_clearance: float,
        tolerance: float,
    ) -> None:
        super(_TerminalGetter, self).__init__()
        self._mesh = mesh
        self._label = label
        self._existing_terminals = list(existing_terminals)
        self._terminal_radius = terminal_radius
        self._minimum_clearance = minimum_clearance
        self._tolerance = tolerance
        self.protrude_toggle = ric.OptionToggle(False, "Flush", "Protrude")

    def OnDynamicDraw(self, e: ri.GetPointDrawEventArgs) -> None:
        super(_TerminalGetter, self).OnDynamicDraw(e)

        for terminal in self._existing_terminals:
            e.Display.DrawPoint(terminal.surface_point, Rhino.Display.PointStyle.X, 6, System.Drawing.Color.Gold)

        preview = _create_terminal(
            self._mesh,
            self._label,
            e.CurrentPoint,
            bool(self.protrude_toggle.CurrentValue),
            self._terminal_radius if self.protrude_toggle.CurrentValue else _mm_to_model(_active_doc(), TERMINAL_LENGTH_MM),
            self._minimum_clearance,
            self._tolerance,
        )
        if preview is None:
            return

        e.Display.DrawPoint(preview.surface_point, Rhino.Display.PointStyle.ControlPoint, 8, System.Drawing.Color.LimeGreen)
        e.Display.DrawLine(
            preview.surface_point,
            preview.anchor_point,
            System.Drawing.Color.Gold,
            2,
        )


def _collect_terminals(
    mesh: rg.Mesh,
    host_brep: rg.Brep,
    terminal_radius: float,
    terminal_length: float,
    minimum_clearance: float,
    tolerance: float,
) -> Optional[Tuple[TerminalPlacement, TerminalPlacement]]:
    terminals: List[TerminalPlacement] = []

    for label in ("Start", "End"):
        gp = _TerminalGetter(mesh, label, terminals, terminal_radius, minimum_clearance, tolerance)
        gp.SetCommandPrompt("Pick the {} terminal on the object".format(label.lower()))
        gp.Constrain(mesh, False)
        gp.AddOptionToggle("TerminalMode", gp.protrude_toggle)

        while True:
            result = gp.Get()
            if result == ri.GetResult.Option:
                continue
            if result == ri.GetResult.Cancel:
                return None
            if result != ri.GetResult.Point:
                return None

            protrudes = bool(gp.protrude_toggle.CurrentValue)
            anchor_depth = terminal_radius if protrudes else terminal_length
            terminal = _create_terminal(
                mesh,
                label,
                gp.Point(),
                protrudes,
                anchor_depth,
                minimum_clearance,
                tolerance,
            )
            if terminal is None:
                Rhino.RhinoApp.WriteLine("That terminal does not fit on the selected surface.")
                continue

            if _terminal_conductive_breps(terminal, host_brep, terminal_radius, terminal_length, tolerance) is None:
                Rhino.RhinoApp.WriteLine(
                    "The {} terminal cylinder does not fit on the selected surface. Choose a flatter or larger area.".format(
                        label.lower()
                    )
                )
                continue

            terminals.append(terminal)
            break

    return terminals[0], terminals[1]


def _collect_touch_nodes(
    mesh: rg.Mesh,
    terminals: Sequence[TerminalPlacement],
    node_radius: float,
    terminal_radius: float,
    min_gap: float,
    tolerance: float,
) -> Optional[List[TouchNodePlacement]]:
    touch_nodes: List[TouchNodePlacement] = []

    while True:
        gp = _TouchNodeGetter(
            mesh,
            touch_nodes,
            terminals,
            node_radius,
            terminal_radius,
            min_gap,
            tolerance,
        )
        gp.SetCommandPrompt(
            "Pick a touch node. Press Enter when done"
        )
        gp.Constrain(mesh, False)
        gp.AcceptNothing(True)
        gp.AcceptEnterWhenDone(True)

        result = gp.Get()
        if result == ri.GetResult.Point:
            label = "Node {}".format(len(touch_nodes) + 1)
            candidate = _create_touch_node(
                mesh,
                label,
                gp.Point(),
                node_radius,
                tolerance,
            )
            error = _validate_touch_node(
                candidate,
                touch_nodes,
                terminals,
                node_radius,
                terminal_radius,
                min_gap,
                tolerance,
            )
            if error is not None:
                Rhino.RhinoApp.WriteLine(error)
                continue
            touch_nodes.append(candidate)
            continue
        if result == ri.GetResult.Nothing:
            return touch_nodes
        if result == ri.GetResult.Cancel:
            return None


def _build_valid_grid(
    mesh: rg.Mesh,
    step: float,
    route_clearance: float,
    tolerance: float,
) -> Tuple[Set[GridIndex], GridSpec]:
    bbox = mesh.GetBoundingBox(True)
    x_count = max(1, int(math.ceil((bbox.Max.X - bbox.Min.X) / step)))
    y_count = max(1, int(math.ceil((bbox.Max.Y - bbox.Min.Y) / step)))
    z_count = max(1, int(math.ceil((bbox.Max.Z - bbox.Min.Z) / step)))

    origin = (
        bbox.Min.X + (step * 0.5),
        bbox.Min.Y + (step * 0.5),
        bbox.Min.Z + (step * 0.5),
    )
    grid = GridSpec(origin=origin, step=step)

    valid_cells: Set[GridIndex] = set()
    for ix in range(x_count):
        x = origin[0] + ix * step
        for iy in range(y_count):
            y = origin[1] + iy * step
            for iz in range(z_count):
                z = origin[2] + iz * step
                point = rg.Point3d(x, y, z)
                if not mesh.IsPointInside(point, tolerance, False):
                    continue
                if _mesh_distance(mesh, point) + tolerance < route_clearance:
                    continue
                valid_cells.add((ix, iy, iz))

    return valid_cells, grid


def _grid_point(index: GridIndex, grid: GridSpec) -> rg.Point3d:
    point = index_to_point(index, grid)
    return rg.Point3d(point[0], point[1], point[2])


def _seed_index(point: rg.Point3d, grid: GridSpec) -> GridIndex:
    return (
        int(round((point.X - grid.origin[0]) / grid.step)),
        int(round((point.Y - grid.origin[1]) / grid.step)),
        int(round((point.Z - grid.origin[2]) / grid.step)),
    )


def _find_nearest_valid_cell(
    point: rg.Point3d,
    valid_cells: Set[GridIndex],
    grid: GridSpec,
) -> Optional[GridIndex]:
    seed = _seed_index(point, grid)
    if seed in valid_cells:
        return seed

    best_index: Optional[GridIndex] = None
    best_distance = float("inf")

    for radius in range(1, 50):
        found_in_radius = False
        for ix in range(seed[0] - radius, seed[0] + radius + 1):
            for iy in range(seed[1] - radius, seed[1] + radius + 1):
                for iz in range(seed[2] - radius, seed[2] + radius + 1):
                    if max(abs(ix - seed[0]), abs(iy - seed[1]), abs(iz - seed[2])) != radius:
                        continue
                    candidate = (ix, iy, iz)
                    if candidate not in valid_cells:
                        continue
                    distance = point.DistanceTo(_grid_point(candidate, grid))
                    if distance < best_distance:
                        best_index = candidate
                        best_distance = distance
                        found_in_radius = True
        if found_in_radius:
            return best_index

    for candidate in valid_cells:
        distance = point.DistanceTo(_grid_point(candidate, grid))
        if distance < best_distance:
            best_index = candidate
            best_distance = distance
    return best_index


def _simplify_points(points: Sequence[rg.Point3d], tolerance: float) -> List[rg.Point3d]:
    if len(points) <= 2:
        return list(points)

    simplified = [points[0]]
    for current, nxt in zip(points[1:-1], points[2:]):
        prev = simplified[-1]
        first = current - prev
        second = nxt - current
        if first.Length <= tolerance or second.Length <= tolerance:
            continue
        first.Unitize()
        second.Unitize()
        if (first - second).Length <= 1e-6:
            continue
        simplified.append(current)
    simplified.append(points[-1])
    return simplified


def _segments_to_polyline_points(
    anchor_points: Sequence[rg.Point3d],
    segments: Sequence[Sequence[GridIndex]],
    grid: GridSpec,
    tolerance: float,
) -> List[rg.Point3d]:
    combined: List[rg.Point3d] = [anchor_points[0]]

    for index, segment in enumerate(segments):
        segment_points = [_grid_point(cell, grid) for cell in segment]
        stitched = [anchor_points[index]] + segment_points + [anchor_points[index + 1]]
        if index > 0:
            stitched = stitched[1:]

        for point in stitched:
            if combined and combined[-1].DistanceTo(point) <= tolerance:
                continue
            combined.append(point)

    return _simplify_points(combined, tolerance)


def _distance_between(a: rg.Point3d, b: rg.Point3d) -> float:
    return a.DistanceTo(b)


def _touch_node_distance_tables(
    start: TerminalPlacement,
    touch_nodes: Sequence[TouchNodePlacement],
    end: TerminalPlacement,
) -> Tuple[List[float], List[float], List[List[float]]]:
    node_count = len(touch_nodes)
    distances = [[0.0 for _ in range(node_count)] for _ in range(node_count)]
    start_distances = [0.0 for _ in range(node_count)]
    end_distances = [0.0 for _ in range(node_count)]

    for index, node in enumerate(touch_nodes):
        start_distances[index] = _distance_between(start.anchor_point, node.anchor_point)
        end_distances[index] = _distance_between(node.anchor_point, end.anchor_point)
        for other_index, other in enumerate(touch_nodes):
            distances[index][other_index] = _distance_between(node.anchor_point, other.anchor_point)

    return start_distances, end_distances, distances


def _nodes_from_order_indices(
    touch_nodes: Sequence[TouchNodePlacement], order_indices: Sequence[int]
) -> Tuple[TouchNodePlacement, ...]:
    return tuple(touch_nodes[index] for index in order_indices)


def _minimum_leg_delta_kohm(
    doc: Rhino.RhinoDoc, metrics: NodeOrderMetrics, wire_diameter_mm: float
) -> float:
    if not metrics.touch_leg_lengths:
        return 0.0
    return min(_nominal_resistance_kohm(doc, leg_length, wire_diameter_mm) for leg_length in metrics.touch_leg_lengths)


def _best_metrics_by_threshold(
    doc: Rhino.RhinoDoc,
    candidates: Sequence[Tuple[str, NodeOrderMetrics]],
    minimum_delta_kohm: float,
    wire_diameter_mm: float,
) -> str:
    satisfying: List[Tuple[str, NodeOrderMetrics]] = []
    for strategy, metrics in candidates:
        if _minimum_leg_delta_kohm(doc, metrics, wire_diameter_mm) + 1e-9 >= minimum_delta_kohm:
            satisfying.append((strategy, metrics))

    if satisfying:
        return min(satisfying, key=lambda item: item[1].total_path_length)[0]

    return max(
        candidates,
        key=lambda item: (
            _minimum_leg_delta_kohm(doc, item[1], wire_diameter_mm),
            -item[1].total_path_length,
        ),
    )[0]


def _best_metrics_by_separation(
    doc: Rhino.RhinoDoc,
    candidates: Sequence[Tuple[str, NodeOrderMetrics]],
    wire_diameter_mm: float,
) -> str:
    return max(
        candidates,
        key=lambda item: (
            _minimum_leg_delta_kohm(doc, item[1], wire_diameter_mm),
            -item[1].total_path_length,
        ),
    )[0]


def _ordering_mode_label(mode: str) -> str:
    if mode == ORDER_MODE_THRESHOLD_FIRST:
        return "threshold-first"
    if mode == ORDER_MODE_SHORTEST_PATH_FIRST:
        return "shortest-path-first"
    if mode == ORDER_MODE_MAXIMIZE_SEPARATION:
        return "maximize-separation"
    return mode


def _choose_touch_node_order(
    doc: Rhino.RhinoDoc,
    start: TerminalPlacement,
    touch_nodes: Sequence[TouchNodePlacement],
    end: TerminalPlacement,
    target_leg_length: float,
    minimum_delta_kohm: float,
    wire_diameter_mm: float,
    ordering_mode: str,
) -> TouchNodeOrderDecision:
    if not touch_nodes:
        empty_metrics = NodeOrderMetrics(order=(), total_path_length=0.0, touch_leg_lengths=(), end_leg_length=0.0)
        return TouchNodeOrderDecision(
            ordered_nodes=(),
            selected_strategy=ORDER_MODE_SHORTEST_PATH_FIRST,
            path_nodes=(),
            threshold_nodes=(),
            max_spacing_nodes=(),
            path_metrics=empty_metrics,
            threshold_metrics=empty_metrics,
            max_spacing_metrics=empty_metrics,
        )

    start_distances, end_distances, pair_distances = _touch_node_distance_tables(start, touch_nodes, end)

    path_order = optimize_node_order_for_path(
        start_distances,
        end_distances,
        pair_distances,
        max_exact_nodes=MAX_DP_NODES,
    )
    threshold_order = optimize_node_order_for_target_leg_length(
        start_distances,
        end_distances,
        pair_distances,
        target_leg_length,
        max_exact_nodes=MAX_DP_NODES,
    )
    max_spacing_order = optimize_node_order_for_maximum_spacing(
        start_distances,
        end_distances,
        pair_distances,
        max_exact_nodes=MAX_DP_NODES,
    )

    path_metrics = evaluate_node_order(path_order, start_distances, end_distances, pair_distances)
    threshold_metrics = evaluate_node_order(threshold_order, start_distances, end_distances, pair_distances)
    max_spacing_metrics = evaluate_node_order(max_spacing_order, start_distances, end_distances, pair_distances)

    candidates = [
        (ORDER_MODE_SHORTEST_PATH_FIRST, path_metrics),
        (ORDER_MODE_THRESHOLD_FIRST, threshold_metrics),
        (ORDER_MODE_MAXIMIZE_SEPARATION, max_spacing_metrics),
    ]

    if ordering_mode == ORDER_MODE_THRESHOLD_FIRST:
        selected_strategy = _best_metrics_by_threshold(
            doc,
            candidates,
            minimum_delta_kohm,
            wire_diameter_mm,
        )
    elif ordering_mode == ORDER_MODE_MAXIMIZE_SEPARATION:
        selected_strategy = _best_metrics_by_separation(doc, candidates, wire_diameter_mm)
    else:
        selected_strategy = ORDER_MODE_SHORTEST_PATH_FIRST

    path_nodes = _nodes_from_order_indices(touch_nodes, path_metrics.order)
    threshold_nodes = _nodes_from_order_indices(touch_nodes, threshold_metrics.order)
    max_spacing_nodes = _nodes_from_order_indices(touch_nodes, max_spacing_metrics.order)
    if selected_strategy == ORDER_MODE_THRESHOLD_FIRST:
        ordered_nodes = threshold_nodes
    elif selected_strategy == ORDER_MODE_MAXIMIZE_SEPARATION:
        ordered_nodes = max_spacing_nodes
    else:
        ordered_nodes = path_nodes
    return TouchNodeOrderDecision(
        ordered_nodes=ordered_nodes,
        selected_strategy=selected_strategy,
        path_nodes=path_nodes,
        threshold_nodes=threshold_nodes,
        max_spacing_nodes=max_spacing_nodes,
        path_metrics=path_metrics,
        threshold_metrics=threshold_metrics,
        max_spacing_metrics=max_spacing_metrics,
    )


def _segment_solids(
    doc: Rhino.RhinoDoc,
    points: Sequence[rg.Point3d],
    radius: float,
) -> List[rg.Brep]:
    solids: List[rg.Brep] = []
    for start, end in zip(points[:-1], points[1:]):
        if start.DistanceTo(end) <= doc.ModelAbsoluteTolerance:
            continue
        line_curve = rg.LineCurve(start, end)
        pipes = rg.Brep.CreatePipe(
            line_curve,
            radius,
            False,
            rg.PipeCapMode.Flat,
            True,
            doc.ModelAbsoluteTolerance,
            doc.ModelAngleToleranceRadians,
        )
        if pipes:
            solids.extend(pipe for pipe in pipes if pipe is not None and pipe.IsValid)

    for point in points:
        sphere = rg.Sphere(point, radius).ToBrep()
        if sphere is not None and sphere.IsValid:
            solids.append(sphere)
    return solids


def _touch_node_shape_breps(
    node: TouchNodePlacement,
    node_radius: float,
) -> List[rg.Brep]:
    shapes: List[rg.Brep] = []

    sphere = rg.Sphere(node.center_point, node_radius).ToBrep()
    if sphere is not None and sphere.IsValid:
        shapes.append(sphere)

    return shapes


def _touch_node_conductive_breps(
    touch_nodes: Sequence[TouchNodePlacement],
    host_brep: rg.Brep,
    node_radius: float,
    tolerance: float,
) -> List[rg.Brep]:
    breps: List[rg.Brep] = []
    for node in touch_nodes:
        shapes = _touch_node_shape_breps(node, node_radius)
        if not shapes:
            continue
        unified = _boolean_union(shapes, tolerance)
        trimmed = _boolean_intersection(unified, [host_brep], tolerance)
        breps.extend(trimmed)
    return breps


def _polyline_curve(points: Sequence[rg.Point3d]) -> Optional[rg.PolylineCurve]:
    if len(points) < 2:
        return None
    curve = rg.PolylineCurve(points)
    return curve if curve.IsValid else None


def _add_named_curve(doc: Rhino.RhinoDoc, points: Sequence[rg.Point3d], name: str) -> bool:
    curve = _polyline_curve(points)
    if curve is None:
        return False
    attributes = rd.ObjectAttributes()
    attributes.Name = name
    return doc.Objects.AddCurve(curve, attributes) != System.Guid.Empty


def _add_breps(doc: Rhino.RhinoDoc, breps: Sequence[rg.Brep], name: str) -> bool:
    attributes = rd.ObjectAttributes()
    attributes.Name = name
    added = False
    for brep in breps:
        if doc.Objects.AddBrep(brep, attributes) != System.Guid.Empty:
            added = True
    return added


def _cumulative_anchor_lengths(
    polyline_points: Sequence[rg.Point3d],
    anchor_points: Sequence[rg.Point3d],
    tolerance: float,
) -> List[float]:
    if not polyline_points or not anchor_points:
        return []

    lengths: List[float] = [0.0]
    anchor_index = 1
    cumulative = 0.0

    for start, end in zip(polyline_points[:-1], polyline_points[1:]):
        cumulative += start.DistanceTo(end)
        while anchor_index < len(anchor_points) and end.DistanceTo(anchor_points[anchor_index]) <= tolerance:
            lengths.append(cumulative)
            anchor_index += 1

    while len(lengths) < len(anchor_points):
        lengths.append(cumulative)
    return lengths


def _resistance_range_kohm(
    doc: Rhino.RhinoDoc, length_in_model_units: float, wire_diameter_mm: float
) -> Tuple[float, float]:
    length_mm = _model_to_mm(doc, length_in_model_units)
    diameter_ratio_sq = (PROTO_PASTA_BASE_DIAMETER_MM / wire_diameter_mm) ** 2
    low = PROTO_PASTA_RESISTANCE_KOHM_PER_100MM[0] * diameter_ratio_sq * (length_mm / 100.0)
    high = PROTO_PASTA_RESISTANCE_KOHM_PER_100MM[1] * diameter_ratio_sq * (length_mm / 100.0)
    return low, high


def _nominal_resistance_kohm(
    doc: Rhino.RhinoDoc, length_in_model_units: float, wire_diameter_mm: float
) -> float:
    low, high = _resistance_range_kohm(doc, length_in_model_units, wire_diameter_mm)
    return (low + high) * 0.5


def _reading_delta_length(doc: Rhino.RhinoDoc, delta_kohm: float, wire_diameter_mm: float) -> float:
    mean_kohm_per_100mm = (
        (PROTO_PASTA_RESISTANCE_KOHM_PER_100MM[0] + PROTO_PASTA_RESISTANCE_KOHM_PER_100MM[1])
        * 0.5
        * (PROTO_PASTA_BASE_DIAMETER_MM / wire_diameter_mm) ** 2
    )
    mean_kohm_per_mm = mean_kohm_per_100mm / 100.0
    required_mm = delta_kohm / mean_kohm_per_mm
    return _mm_to_model(doc, required_mm)


def _target_touch_reading_delta_kohm(minimum_delta_kohm: float) -> float:
    return max(minimum_delta_kohm + 4.0, minimum_delta_kohm * 1.35)


def _format_resistor_value(ohms: float) -> str:
    if ohms >= 1000000.0:
        return "{:.2f} Mohm".format(ohms / 1000000.0)
    return "{:.0f} kohm".format(ohms / 1000.0)


def _validate_touch_reading_spacing(
    doc: Rhino.RhinoDoc,
    touch_node_labels: Sequence[str],
    touch_lengths: Sequence[float],
    minimum_delta_kohm: float,
    wire_diameter_mm: float,
) -> Optional[str]:
    if len(touch_lengths) < 2:
        return None

    previous_label = touch_node_labels[0]
    previous_resistance = _nominal_resistance_kohm(doc, touch_lengths[0], wire_diameter_mm)
    for label, length_value in zip(touch_node_labels[1:], touch_lengths[1:]):
        current_resistance = _nominal_resistance_kohm(doc, length_value, wire_diameter_mm)
        delta = current_resistance - previous_resistance
        if delta + 1e-9 < minimum_delta_kohm:
            return (
                "{} and {} are only {:.1f} kohm apart from the start terminal. "
                "This is below the selected minimum threshold of {:.1f} kohm. Increase spacing, reduce node count, lower the threshold for quick iteration, or use a larger host body."
            ).format(previous_label, label, delta, minimum_delta_kohm)
        previous_label = label
        previous_resistance = current_resistance
    return None


def _get_touch_node_flush_diameter_mm() -> Optional[float]:
    getter = ric.GetNumber()
    getter.SetCommandPrompt("Set the conductive pathway node sphere diameter in millimeters")
    getter.SetDefaultNumber(DEFAULT_TOUCH_NODE_FLUSH_DIAMETER_MM)
    getter.AcceptNothing(True)

    while True:
        result = getter.Get()
        if result == ri.GetResult.Cancel:
            return None
        if result == ri.GetResult.Nothing:
            return DEFAULT_TOUCH_NODE_FLUSH_DIAMETER_MM
        if result != ri.GetResult.Number:
            continue

        value = getter.Number()
        if value + 1e-9 < MIN_TOUCH_NODE_FLUSH_DIAMETER_MM:
            Rhino.RhinoApp.WriteLine("Conductive pathway node sphere diameter is too small. It must be at least 0.5 mm.")
            continue
        return value


def _get_conductive_path_diameter_mm(maximum_diameter_mm: float) -> Optional[float]:
    getter = ric.GetNumber()
    getter.SetCommandPrompt("Set the conductive path diameter in millimeters")
    getter.SetDefaultNumber(DEFAULT_WIRE_DIAMETER_MM)
    getter.AcceptNothing(True)

    while True:
        result = getter.Get()
        if result == ri.GetResult.Cancel:
            return None
        if result == ri.GetResult.Nothing:
            value = DEFAULT_WIRE_DIAMETER_MM
        elif result == ri.GetResult.Number:
            value = getter.Number()
        else:
            continue

        if value + 1e-9 < MIN_WIRE_DIAMETER_MM:
            Rhino.RhinoApp.WriteLine("Conductive path diameter is too small. It must be at least 0.5 mm.")
            continue
        if value - 1e-9 > maximum_diameter_mm:
            Rhino.RhinoApp.WriteLine(
                "Conductive path diameter is too large. It cannot exceed the selected conductive pathway node diameter of {:.2f} mm.".format(
                    maximum_diameter_mm
                )
            )
            continue
        return value


def _get_min_touch_reading_delta_kohm() -> Optional[float]:
    getter = ric.GetNumber()
    getter.SetCommandPrompt("Set the minimum conductive pathway node reading separation in kohm")
    getter.SetDefaultNumber(DEFAULT_MIN_TOUCH_READING_DELTA_KOHM)
    getter.AcceptNothing(True)

    while True:
        result = getter.Get()
        if result == ri.GetResult.Cancel:
            return None
        if result == ri.GetResult.Nothing:
            return DEFAULT_MIN_TOUCH_READING_DELTA_KOHM
        if result != ri.GetResult.Number:
            continue

        value = getter.Number()
        if value + 1e-9 < MIN_ALLOWED_TOUCH_READING_DELTA_KOHM:
            Rhino.RhinoApp.WriteLine(
                "Minimum conductive pathway node reading separation is too small. It must be at least 1.0 kohm."
            )
            continue
        return value


def _get_touch_node_ordering_mode() -> Optional[str]:
    getter = ric.GetString()
    getter.SetCommandPrompt(
        "Set touch-node ordering mode: threshold-first, shortest-path-first, or maximize-separation"
    )
    getter.SetDefaultString(ORDER_MODE_THRESHOLD_FIRST)
    getter.AcceptNothing(True)

    aliases = {
        "threshold-first": ORDER_MODE_THRESHOLD_FIRST,
        "threshold": ORDER_MODE_THRESHOLD_FIRST,
        "shortest-path-first": ORDER_MODE_SHORTEST_PATH_FIRST,
        "shortest": ORDER_MODE_SHORTEST_PATH_FIRST,
        "path": ORDER_MODE_SHORTEST_PATH_FIRST,
        "maximize-separation": ORDER_MODE_MAXIMIZE_SEPARATION,
        "maximize": ORDER_MODE_MAXIMIZE_SEPARATION,
        "separation": ORDER_MODE_MAXIMIZE_SEPARATION,
        "max": ORDER_MODE_MAXIMIZE_SEPARATION,
    }

    while True:
        result = getter.Get()
        if result == ri.GetResult.Cancel:
            return None
        if result == ri.GetResult.Nothing:
            return ORDER_MODE_THRESHOLD_FIRST
        if result != ri.GetResult.String:
            continue

        value = getter.StringResult().strip().lower()
        if value in aliases:
            return aliases[value]
        Rhino.RhinoApp.WriteLine(
            "Invalid ordering mode. Use threshold-first, shortest-path-first, or maximize-separation."
        )


def _protected_anchor_cells(
    anchor_cells: Sequence[GridIndex],
    anchor_radii: Sequence[float],
    step: float,
) -> Tuple[Set[GridIndex], int]:
    protected: Set[GridIndex] = set()
    max_radius = 0
    for cell, radius in zip(anchor_cells, anchor_radii):
        cell_radius = max(1, int(math.ceil(radius / step)))
        max_radius = max(max_radius, cell_radius)
        protected.update(dilate_cells({cell}, cell_radius))
    return protected, max_radius


def _add_output_geometry(
    doc: Rhino.RhinoDoc,
    host_brep: rg.Brep,
    polyline_points: Sequence[rg.Point3d],
    touch_nodes: Sequence[TouchNodePlacement],
    terminals: Sequence[TerminalPlacement],
    wire_radius: float,
    node_radius: float,
    terminal_radius: float,
    terminal_length: float,
) -> bool:
    if len(polyline_points) < 2:
        return False

    if not _add_named_curve(doc, polyline_points, "GenerateInternalWire_Centerline"):
        return False

    path_breps = _segment_solids(doc, polyline_points, wire_radius)
    touch_node_breps = _touch_node_conductive_breps(
        touch_nodes,
        host_brep,
        node_radius,
        doc.ModelAbsoluteTolerance,
    )
    terminal_breps: List[rg.Brep] = []
    for terminal in terminals:
        solids = _terminal_conductive_breps(
            terminal,
            host_brep,
            terminal_radius,
            terminal_length,
            doc.ModelAbsoluteTolerance,
        )
        if solids is None:
            return False
        terminal_breps.extend(solids)

    conductive_path = _boolean_union(path_breps + terminal_breps, doc.ModelAbsoluteTolerance)
    conductive_nodes = _boolean_union(touch_node_breps, doc.ModelAbsoluteTolerance)
    conductive_path = _boolean_difference(conductive_path, conductive_nodes, doc.ModelAbsoluteTolerance)

    path_added = _add_breps(doc, conductive_path, "GenerateInternalWire_ConductivePath")
    node_added = _add_breps(doc, conductive_nodes, "GenerateInternalWire_ConductivePathwayNodes")
    doc.Views.Redraw()
    return path_added and node_added


def run_generate_internal_wire() -> Rhino.Commands.Result:
    doc = _active_doc()
    tolerance = doc.ModelAbsoluteTolerance

    casing_thickness = _mm_to_model(doc, CASING_THICKNESS_MM)
    boundary_clearance = _mm_to_model(doc, BOUNDARY_CLEARANCE_MM)
    path_separation = _mm_to_model(doc, PATH_SEPARATION_MM)
    terminal_radius = _mm_to_model(doc, TERMINAL_DIAMETER_MM * 0.5)
    terminal_length = _mm_to_model(doc, TERMINAL_LENGTH_MM)

    objref = _select_target_geometry()
    if objref is None:
        return Rhino.Commands.Result.Cancel

    mesh = _mesh_from_objref(objref, tolerance)
    host_brep = _host_brep_from_objref(objref, tolerance)
    if mesh is None or host_brep is None:
        Rhino.RhinoApp.WriteLine(
            "The selected object must be a closed mesh or a solid that can be converted to a closed solid."
        )
        return Rhino.Commands.Result.Failure

    flush_node_diameter_mm = _get_touch_node_flush_diameter_mm()
    if flush_node_diameter_mm is None:
        return Rhino.Commands.Result.Cancel

    wire_diameter_mm = _get_conductive_path_diameter_mm(flush_node_diameter_mm)
    if wire_diameter_mm is None:
        return Rhino.Commands.Result.Cancel

    wire_radius = _mm_to_model(doc, wire_diameter_mm * 0.5)
    route_clearance = wire_radius + casing_thickness + boundary_clearance

    min_touch_reading_delta_kohm = _get_min_touch_reading_delta_kohm()
    if min_touch_reading_delta_kohm is None:
        return Rhino.Commands.Result.Cancel

    ordering_mode = _get_touch_node_ordering_mode()
    if ordering_mode is None:
        return Rhino.Commands.Result.Cancel

    node_radius = _mm_to_model(doc, flush_node_diameter_mm * 0.5)
    step = _auto_step(mesh, doc, wire_diameter_mm)
    target_touch_reading_delta_kohm = _target_touch_reading_delta_kohm(
        min_touch_reading_delta_kohm
    )
    target_leg_length = _reading_delta_length(doc, target_touch_reading_delta_kohm, wire_diameter_mm)

    terminals = _collect_terminals(
        mesh,
        host_brep,
        terminal_radius,
        terminal_length,
        route_clearance,
        tolerance,
    )
    if terminals is None:
        return Rhino.Commands.Result.Cancel
    start_terminal, end_terminal = terminals

    touch_nodes = _collect_touch_nodes(
        mesh,
        terminals,
        node_radius,
        terminal_radius,
        path_separation,
        tolerance,
    )
    if touch_nodes is None:
        return Rhino.Commands.Result.Cancel

    order_decision = _choose_touch_node_order(
        doc,
        start_terminal,
        touch_nodes,
        end_terminal,
        target_leg_length,
        min_touch_reading_delta_kohm,
        wire_diameter_mm,
        ordering_mode,
    )
    ordered_touch_nodes = list(order_decision.ordered_nodes)
    ordered_labels = [node.label for node in ordered_touch_nodes]

    if ordered_labels:
        path_min_delta = _minimum_leg_delta_kohm(doc, order_decision.path_metrics, wire_diameter_mm)
        threshold_min_delta = _minimum_leg_delta_kohm(doc, order_decision.threshold_metrics, wire_diameter_mm)
        max_spacing_min_delta = _minimum_leg_delta_kohm(doc, order_decision.max_spacing_metrics, wire_diameter_mm)
        if len({
            order_decision.path_metrics.order,
            order_decision.threshold_metrics.order,
            order_decision.max_spacing_metrics.order,
        }) > 1:
            Rhino.RhinoApp.WriteLine(
                "Order logic check: shortest-path-first gives {:.1f} kohm minimum step, threshold-first gives {:.1f} kohm, and maximize-separation gives {:.1f} kohm.".format(
                    path_min_delta,
                    threshold_min_delta,
                    max_spacing_min_delta,
                )
            )
            Rhino.RhinoApp.WriteLine(
                "Using {} mode. Selected order source: {}."
                .format(
                    _ordering_mode_label(ordering_mode),
                    _ordering_mode_label(order_decision.selected_strategy),
                )
            )
        else:
            Rhino.RhinoApp.WriteLine(
                "Using {} mode. All three ordering strategies converged to the same node order at the selected {:.1f} kohm threshold.".format(
                    _ordering_mode_label(ordering_mode),
                    min_touch_reading_delta_kohm,
                )
            )
    if ordered_labels:
        Rhino.RhinoApp.WriteLine(
            "Optimized touch-node order for distinct readings: {}".format(" -> ".join(ordered_labels))
        )

    Rhino.RhinoApp.WriteLine("Building routing grid...")
    valid_cells, grid = _build_valid_grid(mesh, step, route_clearance, tolerance)
    if not valid_cells:
        Rhino.RhinoApp.WriteLine(
            "No valid routing cells were found. The object is not large enough for {:.2f} mm conductive pathing with the retained 0.5 mm casing-equivalent clearance margin and 0.5 mm wall clearance."
            .format(wire_diameter_mm)
        )
        return Rhino.Commands.Result.Failure

    route_points = [start_terminal.anchor_point] + [node.anchor_point for node in ordered_touch_nodes] + [end_terminal.anchor_point]
    route_labels = [start_terminal.label] + [node.label for node in ordered_touch_nodes] + [end_terminal.label]

    node_cells: List[GridIndex] = []
    for point in route_points:
        cell = _find_nearest_valid_cell(point, valid_cells, grid)
        if cell is None:
            Rhino.RhinoApp.WriteLine(
                "A selected terminal or node could not be mapped into the routing volume. Choose a location with more internal space."
            )
            return Rhino.Commands.Result.Failure
        node_cells.append(cell)

    anchor_radii = [terminal_radius] + [node_radius for _ in ordered_touch_nodes] + [terminal_radius]
    reserved_cells, reserved_exemption_radius = _protected_anchor_cells(anchor_cells=node_cells, anchor_radii=anchor_radii, step=step)

    spacing_radius = max(0, int(math.ceil((wire_diameter_mm + PATH_SEPARATION_MM) * _mm_to_model(doc, 1.0) / step)))
    node_exemption_radius = max(
        1,
        int(math.ceil((max(flush_node_diameter_mm, TERMINAL_DIAMETER_MM) * 0.5 + wire_diameter_mm + PATH_SEPARATION_MM) * _mm_to_model(doc, 1.0) / step)),
    )

    try:
        segments = route_node_sequence(
            valid_cells=valid_cells,
            node_sequence=node_cells,
            penalty_radius=0,
            penalty_weight=step,
            blocked_radius=spacing_radius,
            blocked_exemption_radius=node_exemption_radius,
            reserved_cells=reserved_cells,
            reserved_exemption_radius=reserved_exemption_radius,
            node_labels=route_labels,
            allow_diagonals=False,
        )
    except RoutingError as error:
        Rhino.RhinoApp.WriteLine(str(error))
        Rhino.RhinoApp.WriteLine(
            "The pathway between those points is not big enough for {:.2f} mm wiring, the retained 0.5 mm casing-equivalent clearance margin, 0.5 mm spacing, and the selected conductive pathway node sphere diameter."
            .format(wire_diameter_mm)
        )
        return Rhino.Commands.Result.Failure

    polyline_points = _segments_to_polyline_points(route_points, segments, grid, tolerance)
    cumulative_lengths = _cumulative_anchor_lengths(polyline_points, route_points, tolerance)
    touch_lengths = cumulative_lengths[1:-1]
    sensing_error = _validate_touch_reading_spacing(
        doc,
        ordered_labels,
        touch_lengths,
        min_touch_reading_delta_kohm,
        wire_diameter_mm,
    )
    if sensing_error is not None:
        Rhino.RhinoApp.WriteLine(sensing_error)
        Rhino.RhinoApp.WriteLine(
            "The design was rejected so the conductive pathway nodes stay electrically distinguishable when used with Arduino capacitive sensing."
        )
        return Rhino.Commands.Result.Failure

    if not _add_output_geometry(
        doc,
        host_brep,
        polyline_points,
        ordered_touch_nodes,
        terminals,
        wire_radius,
        node_radius,
        terminal_radius,
        terminal_length,
    ):
        Rhino.RhinoApp.WriteLine(
            "Failed to add the conductive path or conductive pathway node solids to the document."
        )
        return Rhino.Commands.Result.Failure

    total_low, total_high = _resistance_range_kohm(doc, cumulative_lengths[-1], wire_diameter_mm)
    Rhino.RhinoApp.WriteLine(
        "Estimated start-to-end conductive resistance at {:.2f} mm diameter with ProtoPasta Conductive PLA: {:.1f}-{:.1f} kohm.".format(
            wire_diameter_mm,
            total_low,
            total_high,
        )
    )
    Rhino.RhinoApp.WriteLine(
        "Suggested Arduino send-pin resistor: {} to {}. Start with 1.00 Mohm."
        .format(
            _format_resistor_value(SUGGESTED_SERIES_RESISTOR_RANGE_OHM[0]),
            _format_resistor_value(SUGGESTED_SERIES_RESISTOR_RANGE_OHM[1]),
        )
    )
    Rhino.RhinoApp.WriteLine(
        "Distinct-touch targeting uses a nominal {:.1f} kohm step per successive conductive pathway node at {:.2f} mm path diameter and rejects anything below the selected {:.1f} kohm threshold."
        .format(
            target_touch_reading_delta_kohm,
            wire_diameter_mm,
            min_touch_reading_delta_kohm,
        )
    )

    for label, length_value in zip(route_labels[1:-1], cumulative_lengths[1:-1]):
        low, high = _resistance_range_kohm(doc, length_value, wire_diameter_mm)
        nominal = _nominal_resistance_kohm(doc, length_value, wire_diameter_mm)
        Rhino.RhinoApp.WriteLine(
            "{} is approximately {:.1f}-{:.1f} kohm from the start terminal (nominal {:.1f} kohm).".format(
                label,
                low,
                high,
                nominal,
            )
        )

    Rhino.RhinoApp.WriteLine(
        "Generated {:.2f} mm conductive pathing with a retained 0.5 mm casing-equivalent clearance margin, 0.5 mm wall clearance, a user-set conductive pathway node sphere diameter of {:.2f} mm, and 3 mm x 6 mm terminal connectors."
        .format(wire_diameter_mm, flush_node_diameter_mm)
    )
    return Rhino.Commands.Result.Success