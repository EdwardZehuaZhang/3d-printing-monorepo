from __future__ import annotations

import itertools
import math
import time
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
    optimize_node_order_for_target_leg_length,
    route_node_sequence,
)


MAX_ESTIMATED_CELLS = 180000
FIXED_WIRE_DIAMETER_XY_MM = 0.8
FIXED_WIRE_DIAMETER_Z_MM = 1.2
CASING_THICKNESS_MM = 0.5
BOUNDARY_CLEARANCE_MM = 0.5
PATH_SEPARATION_MM = 1.0
MIN_INTRA_PATH_GAP_MM = 1.0
DEFAULT_TOUCH_NODE_FLUSH_DIAMETER_MM = 10.0
MIN_TOUCH_NODE_FLUSH_DIAMETER_MM = 0.5
TERMINAL_DIAMETER_MM = 3.0
TERMINAL_LENGTH_MM = 6.0
MAX_DP_NODES = 10
MAX_EXACT_TOUCH_ORDER_NODES = 6
MAX_ORDER_CANDIDATES = 5
DEFAULT_TOUCH_READING_DELTA_KOHM = 10.0
MIN_ALLOWED_TOUCH_READING_DELTA_KOHM = 1.0
SUGGESTED_SERIES_RESISTOR_RANGE_OHM = (470000.0, 2200000.0)
PROTO_PASTA_BASE_DIAMETER_MM = 1.5
PROTO_PASTA_RESISTANCE_KOHM_PER_100MM = (4.8, 5.0)
PRINT_LAYER_HEIGHT_MM = 0.2
LAYER_COMPACTION_VERTICAL_MOVE_PENALTY = 2.0
ROUTER_BUILD_VERSION = "v37 layered logic"
ROUTER_BUILD_DATE = "2026-03-23"
ROUTER_ROUTING_PROFILE = "deterministic-layercake-bridge-lock"
ROUTER_BUILD_SOURCE = "main-source"
ROUTER_BUILD_TAG = "{} | {} | {} | {}".format(
    ROUTER_BUILD_VERSION,
    ROUTER_BUILD_DATE,
    ROUTER_ROUTING_PROFILE,
    ROUTER_BUILD_SOURCE,
)


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
class TouchNodeOrderCandidate:
    ordered_nodes: Tuple[TouchNodePlacement, ...]
    metrics: NodeOrderMetrics
    max_length_error: float
    total_length_error: float


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
    go.SetCommandPrompt("Select one closed solid to route the conductive trace inside")
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


def _auto_step(mesh: rg.Mesh, doc: Rhino.RhinoDoc, wire_diameter_mm: float, node_count: int = 0) -> float:
    bbox = mesh.GetBoundingBox(True)
    diagonal = bbox.Diagonal.Length
    minimum = max(_mm_to_model(doc, wire_diameter_mm), doc.ModelAbsoluteTolerance * 2.0)
    step = max(minimum, diagonal / 100.0)

    # Reduce the cell budget for high node counts so A* calls stay fast.
    effective_max = MAX_ESTIMATED_CELLS
    if node_count > 8:
        effective_max = int(MAX_ESTIMATED_CELLS * 8.0 / node_count)

    estimated = _estimate_grid_cells(bbox, step)
    if estimated > effective_max:
        step *= (estimated / float(effective_max)) ** (1.0 / 3.0)
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


def _target_length_score(
    metrics: NodeOrderMetrics,
    target_leg_length: float,
) -> Tuple[float, float, float]:
    if not metrics.touch_leg_lengths:
        return 0.0, 0.0, metrics.total_path_length

    errors = [abs(leg_length - target_leg_length) for leg_length in metrics.touch_leg_lengths]
    return max(errors), sum(errors), metrics.total_path_length


def _greedy_target_order(
    start_distances: Sequence[float],
    end_distances: Sequence[float],
    pair_distances: Sequence[Sequence[float]],
    target_leg_length: float,
    start_index: int,
) -> Tuple[int, ...]:
    remaining = list(range(len(start_distances)))
    ordered = [start_index]
    remaining.remove(start_index)

    while remaining:
        last_index = ordered[-1]
        next_index = min(
            remaining,
            key=lambda index: (
                abs(pair_distances[last_index][index] - target_leg_length),
                pair_distances[last_index][index],
                end_distances[index],
            ),
        )
        ordered.append(next_index)
        remaining.remove(next_index)

    return tuple(ordered)


def _preferred_bottom_up_order(
    touch_nodes: Sequence[TouchNodePlacement],
) -> Tuple[int, ...]:
    if not touch_nodes:
        return ()

    xs = [node.anchor_point.X for node in touch_nodes]
    ys = [node.anchor_point.Y for node in touch_nodes]
    min_x = min(xs)
    max_x = max(xs)
    min_y = min(ys)
    max_y = max(ys)

    def _edge_distance(index: int) -> float:
        point = touch_nodes[index].anchor_point
        return min(
            abs(point.X - min_x),
            abs(max_x - point.X),
            abs(point.Y - min_y),
            abs(max_y - point.Y),
        )

    z_values = [node.anchor_point.Z for node in touch_nodes]
    all_same_z = (max(z_values) - min(z_values)) <= 1e-9

    ordered_indices = list(range(len(touch_nodes)))
    if all_same_z:
        ordered_indices.sort(
            key=lambda index: (
                _edge_distance(index),
                touch_nodes[index].anchor_point.X,
                touch_nodes[index].anchor_point.Y,
            )
        )
        return tuple(ordered_indices)

    ordered_indices.sort(
        key=lambda index: (
            touch_nodes[index].anchor_point.Z,
            _edge_distance(index),
            touch_nodes[index].anchor_point.X,
            touch_nodes[index].anchor_point.Y,
        )
    )
    return tuple(ordered_indices)


def _target_order_candidates(
    start: TerminalPlacement,
    touch_nodes: Sequence[TouchNodePlacement],
    end: TerminalPlacement,
    target_leg_length: float,
    max_exact_nodes: int = MAX_EXACT_TOUCH_ORDER_NODES,
) -> List[TouchNodeOrderCandidate]:
    if not touch_nodes:
        empty_metrics = NodeOrderMetrics(
            order=(),
            total_path_length=0.0,
            start_leg_length=0.0,
            touch_leg_lengths=(),
            end_leg_length=0.0,
        )
        return [
            TouchNodeOrderCandidate(
                ordered_nodes=(),
                metrics=empty_metrics,
                max_length_error=0.0,
                total_length_error=0.0,
            )
        ]

    start_distances, end_distances, pair_distances = _touch_node_distance_tables(start, touch_nodes, end)

    preferred_bottom_up = _preferred_bottom_up_order(touch_nodes)
    if not preferred_bottom_up:
        return []

    candidate_orders: List[Tuple[int, ...]] = []
    seen: Set[Tuple[int, ...]] = set()

    def _append_order(order: Tuple[int, ...]) -> None:
        if not order:
            return
        if len(order) != len(touch_nodes):
            return
        if len(set(order)) != len(touch_nodes):
            return
        if order in seen:
            return
        seen.add(order)
        candidate_orders.append(order)

    _append_order(preferred_bottom_up)

    _append_order(
        optimize_node_order_for_target_leg_length(
            start_distances,
            end_distances,
            pair_distances,
            target_leg_length,
            max_exact_nodes=max_exact_nodes,
        )
    )

    seed_indices = sorted(
        range(len(touch_nodes)),
        key=lambda index: (
            abs(start_distances[index] - target_leg_length),
            start_distances[index],
            end_distances[index],
        ),
    )
    for start_index in seed_indices[: min(len(seed_indices), max(2, MAX_ORDER_CANDIDATES * 2))]:
        _append_order(
            _greedy_target_order(
                start_distances,
                end_distances,
                pair_distances,
                target_leg_length,
                start_index,
            )
        )

    preferred_mutable = list(preferred_bottom_up)
    for index in range(len(preferred_mutable) - 1):
        swapped = list(preferred_mutable)
        swapped[index], swapped[index + 1] = swapped[index + 1], swapped[index]
        _append_order(tuple(swapped))

    _append_order(tuple(reversed(preferred_bottom_up)))

    scored: List[TouchNodeOrderCandidate] = []
    for order in candidate_orders:
        metrics = evaluate_node_order(order, start_distances, end_distances, pair_distances)
        max_length_error, total_length_error, _ = _target_length_score(metrics, target_leg_length)
        scored.append(
            TouchNodeOrderCandidate(
                ordered_nodes=_nodes_from_order_indices(touch_nodes, order),
                metrics=metrics,
                max_length_error=max_length_error,
                total_length_error=total_length_error,
            )
        )

    ranked = sorted(
        scored,
        key=lambda candidate: (
            candidate.max_length_error,
            candidate.total_length_error,
            candidate.metrics.total_path_length,
        ),
    )

    preferred_candidate = next(
        (candidate for candidate in ranked if candidate.metrics.order == preferred_bottom_up),
        None,
    )
    if preferred_candidate is None:
        return ranked[:MAX_ORDER_CANDIDATES]

    others = [candidate for candidate in ranked if candidate.metrics.order != preferred_bottom_up]
    return [preferred_candidate] + others[: max(0, MAX_ORDER_CANDIDATES - 1)]


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
        return "That touch node is too large for the selected surface location."

    for node in touch_nodes:
        edge_gap = candidate.center_point.DistanceTo(node.center_point) - (node_radius * 2.0)
        if edge_gap + tolerance < min_gap:
            return "That touch node is too large and overlaps another touch node."

    for terminal in terminals:
        edge_gap = candidate.center_point.DistanceTo(terminal.surface_point) - (node_radius + terminal_radius)
        if edge_gap + tolerance < min_gap:
            return "That touch node is too large and overlaps a terminal connector."
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

        for i, node in enumerate(self._touch_nodes):
            e.Display.DrawSphere(rg.Sphere(node.center_point, self._node_radius), System.Drawing.Color.DeepSkyBlue)
            label_point = node.surface_point + node.outward_direction * self._node_radius * 2.0
            e.Display.Draw2dText("Node {}".format(i + 1), System.Drawing.Color.White, rg.Point3d(label_point), False, 16)

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
        initial_protrude: bool = False,
    ) -> None:
        super(_TerminalGetter, self).__init__()
        self._mesh = mesh
        self._label = label
        self._existing_terminals = list(existing_terminals)
        self._terminal_radius = terminal_radius
        self._minimum_clearance = minimum_clearance
        self._tolerance = tolerance
        self.protrude_toggle = ric.OptionToggle(initial_protrude, "Flush", "Protrude")

    def OnDynamicDraw(self, e: ri.GetPointDrawEventArgs) -> None:
        super(_TerminalGetter, self).OnDynamicDraw(e)

        for terminal in self._existing_terminals:
            e.Display.DrawPoint(terminal.surface_point, Rhino.Display.PointStyle.X, 6, System.Drawing.Color.Gold)

        protrude_preview = bool(self.protrude_toggle.CurrentValue)
        preview = _create_terminal(
            self._mesh,
            self._label,
            e.CurrentPoint,
            protrude_preview,
            self._terminal_radius if protrude_preview else _mm_to_model(_active_doc(), TERMINAL_LENGTH_MM),
            self._terminal_radius * 0.5 if protrude_preview else self._minimum_clearance,
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
    start_protrudes: Optional[bool] = None

    for label in ("Start", "End"):
        gp = _TerminalGetter(
            mesh, label, terminals, terminal_radius, minimum_clearance, tolerance,
            initial_protrude=start_protrudes if (label == "End" and start_protrudes is not None) else False,
        )
        gp.SetCommandPrompt("Pick the {} terminal on the outer surface".format(label.lower()))
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
            if label == "Start":
                start_protrudes = protrudes
            anchor_depth = terminal_radius if protrudes else terminal_length
            terminal = _create_terminal(
                mesh,
                label,
                gp.Point(),
                protrudes,
                anchor_depth,
                terminal_radius * 0.5 if protrudes else minimum_clearance,
                tolerance,
            )
            if terminal is None:
                Rhino.RhinoApp.WriteLine("That terminal does not fit there. Pick a flatter area or an area with more internal space.")
                continue

            if _terminal_conductive_breps(terminal, host_brep, terminal_radius, terminal_length, tolerance) is None:
                Rhino.RhinoApp.WriteLine(
                    "The {} terminal does not fit there. Choose a flatter or larger area.".format(
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
        next_number = len(touch_nodes) + 1
        Rhino.RhinoApp.WriteLine("Choose position of Node {}.".format(next_number))
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
            "Pick Node {}. Press Enter when finished adding touch nodes".format(next_number)
        )
        gp.Constrain(mesh, False)
        gp.AcceptNothing(True)
        gp.AcceptEnterWhenDone(True)

        result = gp.Get()
        if result == ri.GetResult.Point:
            label = "Node {}".format(next_number)
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
            Rhino.RhinoApp.WriteLine("Node {} placed.".format(next_number))
            # Add a persistent text dot in the viewport so the user can see the node number.
            dot = rg.TextDot("Node {}".format(next_number), candidate.surface_point)
            dot_attr = rd.ObjectAttributes()
            dot_attr.Name = "GenerateInternalWire_NodeLabel_{}".format(next_number)
            doc = _active_doc()
            doc.Objects.AddTextDot(dot, dot_attr)
            doc.Views.Redraw()
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
    # Pre-compute z coordinates to avoid repeated multiplication.
    z_coords = [origin[2] + iz * step for iz in range(z_count)]
    # Column probe indices: sample a few z-levels to quickly reject
    # columns that lie entirely outside the mesh (e.g. the donut hole).
    probe_set: Set[int] = {0}
    if z_count > 1:
        probe_set.add(z_count - 1)
    if z_count > 2:
        probe_set.add(z_count // 2)
    if z_count > 4:
        probe_set.add(z_count // 4)
        probe_set.add(3 * z_count // 4)

    for ix in range(x_count):
        x = origin[0] + ix * step
        for iy in range(y_count):
            y = origin[1] + iy * step
            # Quick column rejection: if no probe point is inside, skip.
            column_has_interior = False
            for pz in probe_set:
                if mesh.IsPointInside(rg.Point3d(x, y, z_coords[pz]), tolerance, False):
                    column_has_interior = True
                    break
            if not column_has_interior:
                continue

            for iz in range(z_count):
                point = rg.Point3d(x, y, z_coords[iz])
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


def _segment_solids(
    doc: Rhino.RhinoDoc,
    points: Sequence[rg.Point3d],
    radius_xy: float,
    radius_z: float,
) -> List[rg.Brep]:
    solids: List[rg.Brep] = []
    for start, end in zip(points[:-1], points[1:]):
        if start.DistanceTo(end) <= doc.ModelAbsoluteTolerance:
            continue
        dx = end.X - start.X
        dy = end.Y - start.Y
        dz = end.Z - start.Z
        planar_span = math.sqrt(dx * dx + dy * dy)
        radius = radius_z if abs(dz) > planar_span else radius_xy
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

    # Keep junction spheres at horizontal radius to avoid oversized overlap/shorting at joints.
    junction_radius = radius_xy
    for point in points:
        sphere = rg.Sphere(point, junction_radius).ToBrep()
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


def _add_named_points(doc: Rhino.RhinoDoc, points: Sequence[rg.Point3d], name: str) -> int:
    attributes = rd.ObjectAttributes()
    attributes.Name = name
    added = 0
    for point in points:
        if doc.Objects.AddPoint(point, attributes) != System.Guid.Empty:
            added += 1
    return added


def _preview_points_from_cells(cells: Sequence[GridIndex], grid: GridSpec) -> List[rg.Point3d]:
    return [rg.Point3d(*index_to_point(cell, grid)) for cell in cells]


def _draw_infeasible_internal_preview(
    doc: Rhino.RhinoDoc,
    diagnostics: Dict[str, object],
    grid: GridSpec,
) -> None:
    failure_type = diagnostics.get("type")
    if failure_type not in {"strict_internal_target_failure", "internal_route_unreachable"}:
        return

    segment_index = diagnostics.get("segment_index")
    start = diagnostics.get("start")
    goal = diagnostics.get("goal")

    bridge_cells = diagnostics.get("bridge_path_cells")
    if isinstance(bridge_cells, list) and bridge_cells:
        bridge_points = _preview_points_from_cells(bridge_cells, grid)
        _add_named_curve(doc, bridge_points, "RoutingPreview_InternalBridge")

    prefix_segments = diagnostics.get("accepted_prefix_segments_cells")
    prefix_count = 0
    if isinstance(prefix_segments, list):
        for segment in prefix_segments:
            if not isinstance(segment, list) or not segment:
                continue
            segment_points = _preview_points_from_cells(segment, grid)
            if _add_named_curve(doc, segment_points, "RoutingPreview_AcceptedPrefix"):
                prefix_count += 1

    best_candidate = diagnostics.get("best_candidate_path_cells")
    best_candidate_len = 0
    if isinstance(best_candidate, list) and best_candidate:
        best_candidate_len = len(best_candidate)
        candidate_points = _preview_points_from_cells(best_candidate, grid)
        _add_named_curve(doc, candidate_points, "RoutingPreview_FailingSegmentBest")

    fill_zone_cells = diagnostics.get("fill_zone_sample_cells")
    fill_points_added = 0
    if isinstance(fill_zone_cells, list) and fill_zone_cells:
        fill_points = _preview_points_from_cells(fill_zone_cells, grid)
        fill_points_added = _add_named_points(doc, fill_points, "RoutingPreview_InternalFillZoneSample")

    endpoint_points: List[rg.Point3d] = []
    if isinstance(start, tuple) and len(start) == 3:
        endpoint_points.append(rg.Point3d(*index_to_point(start, grid)))
    if isinstance(goal, tuple) and len(goal) == 3:
        endpoint_points.append(rg.Point3d(*index_to_point(goal, grid)))
    if endpoint_points:
        _add_named_points(doc, endpoint_points, "RoutingPreview_InternalPairEndpoints")

    target_length = diagnostics.get("target_length_cells")
    achieved_length = diagnostics.get("achieved_length_cells")
    fill_zone_count = diagnostics.get("fill_zone_cell_count")
    Rhino.RhinoApp.WriteLine(
        "Infeasible-pair preview generated for segment {} ({} -> {}).".format(
            segment_index,
            start,
            goal,
        )
    )
    if isinstance(target_length, (int, float)) and isinstance(achieved_length, (int, float)):
        Rhino.RhinoApp.WriteLine(
            "Preview stats: achieved {:.1f} / target {:.1f} cells.".format(
                float(achieved_length),
                float(target_length),
            )
        )
    if isinstance(fill_zone_count, int):
        Rhino.RhinoApp.WriteLine(
            "Preview stats: fill-zone cells {}, sampled points {}.".format(
                fill_zone_count,
                fill_points_added,
            )
        )
    Rhino.RhinoApp.WriteLine(
        "Preview stats: accepted-prefix segments {}, failing-segment points {}.".format(
            prefix_count,
            best_candidate_len,
        )
    )


def _write_failure_debug(
    diagnostics: Dict[str, object],
    step: float,
) -> None:
    Rhino.RhinoApp.WriteLine("Routing debug (internal segment diagnostics):")

    segment_index = diagnostics.get("segment_index")
    start = diagnostics.get("start")
    goal = diagnostics.get("goal")
    failure_type = diagnostics.get("type")
    Rhino.RhinoApp.WriteLine(
        "  type={} segment={} start={} goal={}".format(
            failure_type,
            segment_index,
            start,
            goal,
        )
    )

    def _fmt_value(key: str, suffix: str = "") -> str:
        value = diagnostics.get(key)
        if isinstance(value, (int, float)):
            return "{:.1f}{}".format(float(value), suffix)
        return "n/a"

    Rhino.RhinoApp.WriteLine(
        "  target={} achieved={} shortfall={} required={} direct={}".format(
            _fmt_value("target_length_cells", " cells"),
            _fmt_value("achieved_length_cells", " cells"),
            _fmt_value("shortfall_cells", " cells"),
            _fmt_value("required_length_cells", " cells"),
            _fmt_value("direct_distance_cells", " cells"),
        )
    )

    ratio = diagnostics.get("required_ratio")
    if isinstance(ratio, (int, float)):
        Rhino.RhinoApp.WriteLine("  strict_ratio={:.2f}".format(float(ratio)))

    Rhino.RhinoApp.WriteLine(
        "  domain valid={} blocked={} penalty={} candidates={} minCand={} maxCand={}".format(
            diagnostics.get("segment_valid_cell_count", "n/a"),
            diagnostics.get("current_blocked_cell_count", "n/a"),
            diagnostics.get("current_penalty_cell_count", "n/a"),
            diagnostics.get("candidate_path_count", "n/a"),
            _fmt_value("candidate_min_length_cells", " cells"),
            _fmt_value("candidate_max_length_cells", " cells"),
        )
    )

    rejection_counts = diagnostics.get("rejection_counts")
    if isinstance(rejection_counts, dict) and rejection_counts:
        Rhino.RhinoApp.WriteLine(
            "  rejects short={} cross={} nonAdjacent={} keepoutReentry={} nodeKeepout={} downstream={}".format(
                rejection_counts.get("strict_too_short", 0),
                rejection_counts.get("cross_segment_overlap", 0),
                rejection_counts.get("non_adjacent_overlap", 0),
                rejection_counts.get("endpoint_keepout_reentry", 0),
                rejection_counts.get("non_endpoint_node_keepout", 0),
                rejection_counts.get("downstream_backtrack_failure", 0),
            )
        )

    accepted_prefix_segments = diagnostics.get("accepted_prefix_segments_cells")
    prefix_segments_count = len(accepted_prefix_segments) if isinstance(accepted_prefix_segments, list) else 0
    best_candidate = diagnostics.get("best_candidate_path_cells")
    best_candidate_points = len(best_candidate) if isinstance(best_candidate, list) else 0
    Rhino.RhinoApp.WriteLine(
        "  partial_route prefix_segments={} failing_segment_points={}".format(
            prefix_segments_count,
            best_candidate_points,
        )
    )

    bridge_cells = diagnostics.get("bridge_path_cells")
    bridge_len = len(bridge_cells) if isinstance(bridge_cells, list) else 0
    Rhino.RhinoApp.WriteLine(
        "  bridge_path_cells={} (~{:.1f} model-units)".format(
            bridge_len,
            bridge_len * step,
        )
    )

    fill_count = diagnostics.get("fill_zone_cell_count")
    fill_layers = diagnostics.get("fill_zone_layer_count")
    Rhino.RhinoApp.WriteLine(
        "  fill_zone_cells={} fill_zone_layers={} sampled_points={}".format(
            fill_count if isinstance(fill_count, int) else "n/a",
            fill_layers if isinstance(fill_layers, int) else "n/a",
            len(diagnostics.get("fill_zone_sample_cells", [])) if isinstance(diagnostics.get("fill_zone_sample_cells"), list) else "n/a",
        )
    )


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


def _format_length_mm(length_mm: float) -> str:
    if length_mm >= 1000.0:
        return "{:.2f} m".format(length_mm / 1000.0)
    return "{:.1f} mm".format(length_mm)


def _polyline_vertical_span(doc: Rhino.RhinoDoc, points: Sequence[rg.Point3d]) -> float:
    if not points:
        return 0.0
    z_values = [point.Z for point in points]
    return _model_to_mm(doc, max(z_values) - min(z_values))


def _vertical_layer_height_span(points: Sequence[rg.Point3d], doc: Rhino.RhinoDoc) -> int:
    vertical_span_mm = _polyline_vertical_span(doc, points)
    return max(1, int(math.ceil(vertical_span_mm / PRINT_LAYER_HEIGHT_MM)))


def _format_resistor_value(ohms: float) -> str:
    if ohms >= 1000000.0:
        return "{:.2f} Mohm".format(ohms / 1000000.0)
    return "{:.0f} kohm".format(ohms / 1000.0)


def _touch_segment_lengths_from_cumulative_lengths(
    cumulative_lengths: Sequence[float],
) -> List[float]:
    return [
        current - previous
        for previous, current in zip(cumulative_lengths[1:-2], cumulative_lengths[2:-1])
    ]


def _write_touch_step_report(
    doc: Rhino.RhinoDoc,
    touch_node_labels: Sequence[str],
    touch_segment_lengths: Sequence[float],
    wire_diameter_mm: float,
    heading: str,
) -> None:
    if len(touch_node_labels) < 2 or len(touch_segment_lengths) < 1:
        return

    Rhino.RhinoApp.WriteLine(heading)
    for i, seg_length in enumerate(touch_segment_lengths):
        label_a = touch_node_labels[i]
        label_b = touch_node_labels[i + 1]
        length_mm = _model_to_mm(doc, seg_length)
        low, high = _resistance_range_kohm(doc, seg_length, wire_diameter_mm)
        Rhino.RhinoApp.WriteLine(
            "  {} - {} (length: {}, resistance: {:.1f}-{:.1f} kohm)".format(
                label_a, label_b, _format_length_mm(length_mm), low, high
            )
        )

    nominals = [
        _nominal_resistance_kohm(doc, seg, wire_diameter_mm)
        for seg in touch_segment_lengths
    ]
    min_nominal = min(nominals)
    Rhino.RhinoApp.WriteLine(
        "Minimum resistance between touch nodes: {:.1f} kohm ({} pairs).".format(
            min_nominal, len(nominals)
        )
    )


def _get_touch_node_flush_diameter_mm() -> Optional[float]:
    getter = ric.GetNumber()
    getter.SetCommandPrompt("Set the touch node diameter in millimeters")
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
            Rhino.RhinoApp.WriteLine("The touch node diameter is too small. It must be at least 0.5 mm.")
            continue
        return value


def _recommended_touch_reading_delta_kohm(wire_diameter_mm: float) -> float:
    _ = wire_diameter_mm
    return DEFAULT_TOUCH_READING_DELTA_KOHM


def _get_target_touch_reading_delta_kohm(wire_diameter_mm: float) -> Optional[float]:
    recommended_delta_kohm = _recommended_touch_reading_delta_kohm(wire_diameter_mm)
    getter = ric.GetNumber()
    getter.SetCommandPrompt("Set the target resistance difference between neighboring touch nodes in kohm")
    getter.SetDefaultNumber(recommended_delta_kohm)
    getter.AcceptNothing(True)

    while True:
        result = getter.Get()
        if result == ri.GetResult.Cancel:
            return None
        if result == ri.GetResult.Nothing:
            return recommended_delta_kohm
        if result != ri.GetResult.Number:
            continue

        value = getter.Number()
        if value + 1e-9 < MIN_ALLOWED_TOUCH_READING_DELTA_KOHM:
            Rhino.RhinoApp.WriteLine(
                "The target resistance difference is too small. It must be at least 1.0 kohm."
            )
            continue
        return value


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


def _segment_target_lengths(
    route_points: Sequence[rg.Point3d],
    target_touch_leg_length: float,
) -> List[Optional[float]]:
    targets: List[Optional[float]] = [None for _ in range(max(0, len(route_points) - 1))]
    for segment_index in range(1, max(1, len(route_points) - 2)):
        if segment_index >= len(targets) - 1:
            break
        targets[segment_index] = target_touch_leg_length
    return targets


def _add_output_geometry(
    doc: Rhino.RhinoDoc,
    host_brep: rg.Brep,
    polyline_points: Sequence[rg.Point3d],
    touch_nodes: Sequence[TouchNodePlacement],
    terminals: Sequence[TerminalPlacement],
    wire_radius_xy: float,
    wire_radius_z: float,
    node_radius: float,
    terminal_radius: float,
    terminal_length: float,
) -> bool:
    if len(polyline_points) < 2:
        return False

    if not _add_named_curve(doc, polyline_points, "GenerateInternalWire_Centerline"):
        return False

    path_breps = _segment_solids(doc, polyline_points, wire_radius_xy, wire_radius_z)
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
    Rhino.RhinoApp.WriteLine("GenerateInternalWire build: {}".format(ROUTER_BUILD_TAG))

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
            "The selected object must be a closed mesh or a solid that Rhino can treat as a closed volume."
        )
        return Rhino.Commands.Result.Failure

    flush_node_diameter_mm = _get_touch_node_flush_diameter_mm()
    if flush_node_diameter_mm is None:
        return Rhino.Commands.Result.Cancel

    wire_diameter_xy_mm = FIXED_WIRE_DIAMETER_XY_MM
    wire_diameter_z_mm = FIXED_WIRE_DIAMETER_Z_MM
    wire_diameter_for_clearance_mm = max(wire_diameter_xy_mm, wire_diameter_z_mm)
    wire_diameter_for_estimate_mm = wire_diameter_xy_mm
    if flush_node_diameter_mm + 1e-9 < wire_diameter_for_clearance_mm:
        Rhino.RhinoApp.WriteLine(
            "The touch node diameter is too small for fixed conductive trace widths. Set touch node diameter to at least {:.1f} mm.".format(
                wire_diameter_for_clearance_mm
            )
        )
        return Rhino.Commands.Result.Failure

    Rhino.RhinoApp.WriteLine(
        "Conductive trace widths are fixed: {:.1f} mm (XY) and {:.1f} mm (Z).".format(
            wire_diameter_xy_mm,
            wire_diameter_z_mm,
        )
    )

    wire_radius_xy = _mm_to_model(doc, wire_diameter_xy_mm * 0.5)
    wire_radius_z = _mm_to_model(doc, wire_diameter_z_mm * 0.5)
    wire_radius_for_clearance = _mm_to_model(doc, wire_diameter_for_clearance_mm * 0.5)
    route_clearance = wire_radius_for_clearance + casing_thickness + boundary_clearance

    target_touch_reading_delta_kohm = _get_target_touch_reading_delta_kohm(wire_diameter_for_estimate_mm)
    if target_touch_reading_delta_kohm is None:
        return Rhino.Commands.Result.Cancel

    node_radius = _mm_to_model(doc, flush_node_diameter_mm * 0.5)
    target_leg_length = _reading_delta_length(
        doc,
        target_touch_reading_delta_kohm,
        wire_diameter_for_estimate_mm,
    )
    target_leg_length_mm = _model_to_mm(doc, target_leg_length)

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

    step = _auto_step(mesh, doc, wire_diameter_for_clearance_mm, node_count=len(touch_nodes))

    Rhino.RhinoApp.WriteLine("Routing...")
    valid_cells, grid = _build_valid_grid(mesh, step, route_clearance, tolerance)
    if not valid_cells:
        Rhino.RhinoApp.WriteLine(
            "No valid routing space was found. The part is too small for a {:.2f} mm conductive trace while also keeping {:.1f} mm clearance from the outer wall and {:.1f} mm spacing around the trace."
            .format(wire_diameter_for_clearance_mm, BOUNDARY_CLEARANCE_MM, PATH_SEPARATION_MM)
        )
        return Rhino.Commands.Result.Failure

    Rhino.RhinoApp.WriteLine("Ranking touch-node orders...")
    order_candidates = _target_order_candidates(
        start_terminal,
        touch_nodes,
        end_terminal,
        target_leg_length,
    )

    # dilate_cells blocks cells within Chebyshev radius R of each path
    # cell, so the next path starts at distance R+1.  We need
    # (R+1)*step >= wire_diameter + PATH_SEPARATION for the required
    # edge-to-edge gap, i.e.  R >= (wire_d + sep) / step - 1.
    spacing_radius = max(1, int(math.ceil(
        (wire_diameter_for_clearance_mm + PATH_SEPARATION_MM) * _mm_to_model(doc, 1.0) / step - 1.0
    )))
    # The node exemption covers the physical node radius so paths can
    # reach the node through blocked zones, but no larger — oversized
    # exemption zones allow parallel paths with no inter-segment gap.
    node_exemption_radius = spacing_radius

    # Minimum intra-path spacing for U-turns and serpentine fills.
    # ceil((wire_d + gap) / step) gives the minimum center-to-center
    # distance in grid cells so that edge-to-edge gap >= MIN_INTRA_PATH_GAP_MM.
    min_self_spacing = max(1, int(math.ceil(
        (wire_diameter_for_clearance_mm + MIN_INTRA_PATH_GAP_MM) * _mm_to_model(doc, 1.0) / step
    )))

    selected_candidate: Optional[TouchNodeOrderCandidate] = None
    selected_touch_nodes: List[TouchNodePlacement] = []
    selected_route_points: List[rg.Point3d] = []
    selected_route_labels: List[str] = []
    selected_segments: List[List[GridIndex]] = []
    first_failure_reason: Optional[str] = None
    first_failure_order: Optional[List[str]] = None
    first_failure_diagnostics: Optional[Dict[str, object]] = None
    attempted_orders = 0

    for candidate in order_candidates:
        ordered_touch_nodes = list(candidate.ordered_nodes)
        route_points = [start_terminal.anchor_point] + [node.anchor_point for node in ordered_touch_nodes] + [end_terminal.anchor_point]
        route_labels = [start_terminal.label] + [node.label for node in ordered_touch_nodes] + [end_terminal.label]
        # Convert target_leg_length from model units (mm) to grid-cell
        # units so that _path_length() comparisons inside core.py are
        # consistent — _path_length() returns Euclidean distance
        # in grid-index space where each step equals 1.0.
        target_leg_length_cells = target_leg_length / step
        segment_target_lengths = _segment_target_lengths(route_points, target_leg_length_cells)

        node_cells: List[GridIndex] = []
        mapping_failed = False
        for point in route_points:
            cell = _find_nearest_valid_cell(point, valid_cells, grid)
            if cell is None:
                mapping_failed = True
                break
            node_cells.append(cell)
        if mapping_failed:
            Rhino.RhinoApp.WriteLine(
                "A selected terminal or touch node could not be placed into the usable internal volume. Choose a location with more internal space."
            )
            return Rhino.Commands.Result.Failure

        anchor_radii = [terminal_radius] + [node_radius for _ in ordered_touch_nodes] + [terminal_radius]
        reserved_cells, reserved_exemption_radius = _protected_anchor_cells(
            anchor_cells=node_cells,
            anchor_radii=anchor_radii,
            step=step,
        )

        attempted_orders += 1
        per_order_budget = max(60.0, 300.0 / max(1, len(order_candidates)))
        deadline = time.monotonic() + per_order_budget
        try:
            segments = route_node_sequence(
                valid_cells=valid_cells,
                node_sequence=node_cells,
                segment_target_lengths=segment_target_lengths,
                penalty_radius=0,
                penalty_weight=step,
                blocked_radius=spacing_radius,
                blocked_exemption_radius=node_exemption_radius,
                reserved_cells=reserved_cells,
                reserved_exemption_radius=reserved_exemption_radius,
                node_labels=route_labels,
                allow_diagonals=False,
                vertical_move_penalty=LAYER_COMPACTION_VERTICAL_MOVE_PENALTY,
                deadline=deadline,
                min_self_spacing=min_self_spacing,
            )
        except RoutingError as error:
            attempted_sequence = [start_terminal.label] + [node.label for node in ordered_touch_nodes] + [end_terminal.label]
            Rhino.RhinoApp.WriteLine(
                "Order attempt failed: {} | reason: {}".format(
                    " -> ".join(attempted_sequence),
                    str(error),
                )
            )
            if first_failure_reason is None:
                first_failure_reason = str(error)
                first_failure_order = [node.label for node in ordered_touch_nodes]
                first_failure_diagnostics = error.diagnostics if hasattr(error, "diagnostics") else None
            continue

        selected_candidate = candidate
        selected_touch_nodes = ordered_touch_nodes
        selected_route_points = route_points
        selected_route_labels = route_labels
        selected_segments = segments
        break

    if selected_candidate is None:
        if first_failure_reason is not None and first_failure_order is not None:
            Rhino.RhinoApp.WriteLine(
                "No touch-node order could be routed after checking {} candidate order(s).".format(
                    attempted_orders
                )
            )
            Rhino.RhinoApp.WriteLine(
                "The closest match to the target was {}. It failed because {}".format(
                    " -> ".join(first_failure_order),
                    first_failure_reason.rstrip("."),
                )
            )
            Rhino.RhinoApp.WriteLine(
                "This does not always mean the whole part is too small. It means at least one segment did not have enough local space once the trace diameter, node protection zones, wall clearance, and trace spacing were applied."
            )
            if first_failure_diagnostics:
                _draw_infeasible_internal_preview(doc, first_failure_diagnostics, grid)
                _write_failure_debug(first_failure_diagnostics, step)
        else:
            Rhino.RhinoApp.WriteLine(
                "No valid route could be found with the current geometry and spacing rules."
            )
        return Rhino.Commands.Result.Failure

    ordered_touch_nodes = selected_touch_nodes
    ordered_labels = [node.label for node in ordered_touch_nodes]

    # Print the routed node path sequence.
    path_sequence = [start_terminal.label] + ordered_labels + [end_terminal.label]
    Rhino.RhinoApp.WriteLine("Routed path: {}".format(" -> ".join(path_sequence)))

    polyline_points = _segments_to_polyline_points(selected_route_points, selected_segments, grid, tolerance)
    cumulative_lengths = _cumulative_anchor_lengths(polyline_points, selected_route_points, tolerance)
    touch_segment_lengths = _touch_segment_lengths_from_cumulative_lengths(cumulative_lengths)
    vertical_span_mm = _polyline_vertical_span(doc, polyline_points)
    vertical_layer_span = _vertical_layer_height_span(polyline_points, doc)
    if ordered_labels:
        _write_touch_step_report(
            doc,
            ordered_labels,
            touch_segment_lengths,
            wire_diameter_for_estimate_mm,
            "Resistance between touch nodes:",
        )

    if not _add_output_geometry(
        doc,
        host_brep,
        polyline_points,
        ordered_touch_nodes,
        terminals,
        wire_radius_xy,
        wire_radius_z,
        node_radius,
        terminal_radius,
        terminal_length,
    ):
        Rhino.RhinoApp.WriteLine(
            "The route was calculated, but Rhino could not add the conductive trace or touch-node solids to the document."
        )
        return Rhino.Commands.Result.Failure

    total_low, total_high = _resistance_range_kohm(
        doc,
        cumulative_lengths[-1],
        wire_diameter_for_estimate_mm,
    )
    Rhino.RhinoApp.WriteLine(
        "Total resistance: {:.1f}-{:.1f} kohm.".format(total_low, total_high)
    )
    return Rhino.Commands.Result.Success