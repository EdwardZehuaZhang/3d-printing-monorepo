from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Set, Tuple

import Rhino
import Rhino.DocObjects as rd
import Rhino.Geometry as rg
import Rhino.Input as ri
import Rhino.Input.Custom as ric
import System
import System.Drawing
import scriptcontext as sc

from .core import GridIndex, GridSpec, RoutingError, index_to_point, route_node_sequence


MAX_ESTIMATED_CELLS = 180000
MIN_NODE_DIAMETER_MM = 0.5
MIN_WIRE_DIAMETER_MM = 0.5
MIN_NODE_GAP_MM = 0.5
MIN_CASING_THICKNESS_MM = 0.5


@dataclass(frozen=True)
class RoutingOptions:
    step: float
    clearance: float
    wire_radius: float
    node_radius: float


@dataclass(frozen=True)
class NodePlacement:
    surface_point: rg.Point3d
    center_point: rg.Point3d
    inward_direction: rg.Vector3d


def _active_doc() -> Rhino.RhinoDoc:
    if "__rhino_doc__" in globals() and __rhino_doc__ is not None:
        return __rhino_doc__
    return sc.doc


def _mm_to_model(doc: Rhino.RhinoDoc, value_mm: float) -> float:
    return value_mm * Rhino.RhinoMath.UnitScale(
        Rhino.UnitSystem.Millimeters, doc.ModelUnitSystem
    )


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


def _prompt_for_options(mesh: rg.Mesh, doc: Rhino.RhinoDoc) -> Optional[RoutingOptions]:
    bbox = mesh.GetBoundingBox(True)
    diagonal = bbox.Diagonal.Length
    minimum = max(doc.ModelAbsoluteTolerance * 2.0, diagonal / 1000.0)

    min_clearance = _mm_to_model(doc, MIN_CASING_THICKNESS_MM)
    min_wire_radius = _mm_to_model(doc, MIN_WIRE_DIAMETER_MM * 0.5)
    min_node_diameter = _mm_to_model(doc, MIN_NODE_DIAMETER_MM)

    default_step = max(minimum, diagonal / 55.0)
    default_clearance = max(min_clearance, default_step * 0.75)
    default_wire_radius = max(min_wire_radius, default_step * 0.35)
    default_node_diameter = max(min_node_diameter, default_wire_radius * 4.0)

    get_option = ric.GetOption()
    get_option.SetCommandPrompt("Set routing options. Press Enter to continue")
    get_option.AcceptNothing(True)
    get_option.AcceptEnterWhenDone(True)

    voxel_option = ric.OptionDouble(
        default_step,
        minimum,
        max(default_step * 20.0, minimum * 2.0),
    )
    clearance_option = ric.OptionDouble(
        default_clearance,
        min_clearance,
        max(diagonal / 2.0, default_clearance + minimum),
    )
    wire_option = ric.OptionDouble(
        default_wire_radius,
        min_wire_radius,
        max(diagonal / 4.0, default_wire_radius + minimum),
    )
    node_option = ric.OptionDouble(
        default_node_diameter,
        min_node_diameter,
        max(diagonal / 2.0, default_node_diameter + minimum),
    )

    get_option.AddOptionDouble("VoxelSize", voxel_option)
    get_option.AddOptionDouble("Clearance", clearance_option)
    get_option.AddOptionDouble("WireRadius", wire_option)
    get_option.AddOptionDouble("NodeSize", node_option)

    while True:
        result = get_option.Get()
        if result == ri.GetResult.Option:
            continue
        if result == ri.GetResult.Nothing:
            step = voxel_option.CurrentValue
            estimated = _estimate_grid_cells(bbox, step)
            if estimated > MAX_ESTIMATED_CELLS:
                adjusted_step = step * ((estimated / float(MAX_ESTIMATED_CELLS)) ** (1.0 / 3.0))
                Rhino.RhinoApp.WriteLine(
                    "VoxelSize increased from {:.4f} to {:.4f} to keep the routing grid manageable.".format(
                        step, adjusted_step
                    )
                )
                step = adjusted_step

            return RoutingOptions(
                step=step,
                clearance=max(clearance_option.CurrentValue, min_clearance),
                wire_radius=max(wire_option.CurrentValue, min_wire_radius),
                node_radius=max(node_option.CurrentValue * 0.5, min_node_diameter * 0.5),
            )
        return None


def _mesh_distance(mesh: rg.Mesh, point: rg.Point3d) -> float:
    mesh_point = mesh.ClosestMeshPoint(point, 0.0)
    if mesh_point is not None:
        return point.DistanceTo(mesh.PointAt(mesh_point))
    return point.DistanceTo(mesh.ClosestPoint(point))


def _create_node_placement(
    mesh: rg.Mesh,
    surface_point: rg.Point3d,
    node_radius: float,
    tolerance: float,
) -> Optional[NodePlacement]:
    mesh_point = mesh.ClosestMeshPoint(surface_point, 0.0)
    if mesh_point is None:
        return None

    normal = mesh.NormalAt(mesh_point)
    if not normal.Unitize():
        return None

    candidates: List[NodePlacement] = []
    for scale in (-1.0, 1.0):
        direction = rg.Vector3d(normal)
        direction *= scale
        center = surface_point + (direction * node_radius)
        if not mesh.IsPointInside(center, tolerance, False):
            continue
        if _mesh_distance(mesh, center) + tolerance < node_radius:
            continue
        inward_direction = rg.Vector3d(direction)
        inward_direction.Unitize()
        candidates.append(
            NodePlacement(
                surface_point=rg.Point3d(surface_point),
                center_point=center,
                inward_direction=inward_direction,
            )
        )

    if not candidates:
        return None
    return max(candidates, key=lambda candidate: _mesh_distance(mesh, candidate.center_point))


def _validate_node_placement(
    candidate: Optional[NodePlacement],
    nodes: Sequence[NodePlacement],
    node_radius: float,
    min_gap: float,
    tolerance: float,
) -> Optional[str]:
    if (node_radius * 2.0) + tolerance < min_gap:
        return "NodeSize must be at least 0.5 mm."

    if candidate is None:
        return "NodeSize is too large for this location on the object."

    for index, node in enumerate(nodes, start=1):
        edge_gap = candidate.center_point.DistanceTo(node.center_point) - (node_radius * 2.0)
        if edge_gap + tolerance < min_gap:
            return "Node {} is too close. Node solids must stay at least 0.5 mm apart.".format(index)
    return None


class _NodePointGetter(ric.GetPoint):
    def __init__(
        self,
        mesh: rg.Mesh,
        nodes: Sequence[NodePlacement],
        node_radius: float,
        tolerance: float,
        min_gap: float,
    ) -> None:
        super(_NodePointGetter, self).__init__()
        self._mesh = mesh
        self._nodes = list(nodes)
        self._node_radius = node_radius
        self._tolerance = tolerance
        self._min_gap = min_gap

    def OnDynamicDraw(self, e: ri.GetPointDrawEventArgs) -> None:
        super(_NodePointGetter, self).OnDynamicDraw(e)

        accepted_color = System.Drawing.Color.DeepSkyBlue
        for node in self._nodes:
            e.Display.DrawSphere(rg.Sphere(node.center_point, self._node_radius), accepted_color)

        preview = _create_node_placement(
            self._mesh,
            e.CurrentPoint,
            self._node_radius,
            self._tolerance,
        )
        preview_error = _validate_node_placement(
            preview,
            self._nodes,
            self._node_radius,
            self._min_gap,
            self._tolerance,
        )

        if preview is None:
            return

        preview_color = (
            System.Drawing.Color.OrangeRed
            if preview_error is not None
            else System.Drawing.Color.LimeGreen
        )
        e.Display.DrawSphere(rg.Sphere(preview.center_point, self._node_radius), preview_color)

        if self._nodes:
            e.Display.DrawLine(
                self._nodes[-1].center_point,
                preview.center_point,
                System.Drawing.Color.Gold,
                2,
            )


def _collect_nodes(
    mesh: rg.Mesh,
    node_radius: float,
    tolerance: float,
    min_gap: float,
) -> Optional[List[NodePlacement]]:
    nodes: List[NodePlacement] = []

    while True:
        gp = _NodePointGetter(mesh, nodes, node_radius, tolerance, min_gap)
        gp.SetCommandPrompt(
            "Pick node point {}. Press Enter when done".format(len(nodes) + 1)
        )
        gp.Constrain(mesh, False)
        gp.AcceptNothing(len(nodes) >= 2)
        gp.AcceptEnterWhenDone(True)
        if nodes:
            gp.SetBasePoint(nodes[-1].center_point, False)
            gp.DrawLineFromPoint(nodes[-1].center_point, False)

        result = gp.Get()
        if result == ri.GetResult.Point:
            placement = _create_node_placement(mesh, gp.Point(), node_radius, tolerance)
            error = _validate_node_placement(placement, nodes, node_radius, min_gap, tolerance)
            if error is not None:
                Rhino.RhinoApp.WriteLine(error)
                continue
            nodes.append(placement)
            continue
        if result == ri.GetResult.Nothing and len(nodes) >= 2:
            return nodes
        if result == ri.GetResult.Cancel:
            return None
        Rhino.RhinoApp.WriteLine("Pick at least two valid node points.")


def _build_valid_grid(
    mesh: rg.Mesh,
    step: float,
    clearance: float,
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
                if clearance > 0.0 and _mesh_distance(mesh, point) + tolerance < clearance:
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

    for radius in range(1, 40):
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
    node_points: Sequence[rg.Point3d],
    segments: Sequence[Sequence[GridIndex]],
    grid: GridSpec,
    tolerance: float,
) -> List[rg.Point3d]:
    combined: List[rg.Point3d] = [node_points[0]]

    for index, segment in enumerate(segments):
        segment_points = [_grid_point(cell, grid) for cell in segment]
        stitched = [node_points[index]] + segment_points + [node_points[index + 1]]
        if index > 0:
            stitched = stitched[1:]

        for point in stitched:
            if combined and combined[-1].DistanceTo(point) <= tolerance:
                continue
            combined.append(point)

    return _simplify_points(combined, tolerance)


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
        sphere = rg.Sphere(point, radius)
        brep = sphere.ToBrep()
        if brep is not None and brep.IsValid:
            solids.append(brep)

    return solids


def _node_solids(nodes: Sequence[NodePlacement], node_radius: float) -> List[rg.Brep]:
    solids: List[rg.Brep] = []
    for node in nodes:
        brep = rg.Sphere(node.center_point, node_radius).ToBrep()
        if brep is not None and brep.IsValid:
            solids.append(brep)
    return solids


def _boolean_union(breps: Sequence[rg.Brep], tolerance: float) -> List[rg.Brep]:
    valid = [brep for brep in breps if brep is not None and brep.IsValid]
    if not valid:
        return []
    if len(valid) == 1:
        return valid

    union = rg.Brep.CreateBooleanUnion(valid, tolerance)
    if union:
        return list(union)
    return valid


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
    if difference:
        return list(difference)
    return valid_minuend


def _add_breps(doc: Rhino.RhinoDoc, breps: Sequence[rg.Brep], name: str) -> bool:
    attributes = rd.ObjectAttributes()
    attributes.Name = name
    added = False
    for brep in breps:
        if doc.Objects.AddBrep(brep, attributes) != System.Guid.Empty:
            added = True
    return added


def _add_output_geometry(
    doc: Rhino.RhinoDoc,
    polyline_points: Sequence[rg.Point3d],
    nodes: Sequence[NodePlacement],
    wire_radius: float,
    node_radius: float,
    casing_thickness: float,
) -> bool:
    if len(polyline_points) < 2:
        return False

    if not _add_named_curve(doc, polyline_points, "GenerateInternalWire_Centerline"):
        return False

    path_union = _boolean_union(_segment_solids(doc, polyline_points, wire_radius), doc.ModelAbsoluteTolerance)
    if not path_union:
        return False

    node_union = _boolean_union(_node_solids(nodes, node_radius), doc.ModelAbsoluteTolerance)
    if not node_union:
        return False

    path_without_nodes = _boolean_difference(path_union, node_union, doc.ModelAbsoluteTolerance)
    if path_without_nodes:
        path_union = path_without_nodes

    outer_radius = wire_radius + casing_thickness
    casing_outer = _boolean_union(_segment_solids(doc, polyline_points, outer_radius), doc.ModelAbsoluteTolerance)
    if not casing_outer:
        return False

    casing_shell = _boolean_difference(
        casing_outer,
        list(path_union) + list(node_union),
        doc.ModelAbsoluteTolerance,
    )

    path_added = _add_breps(doc, path_union, "GenerateInternalWire_ConductivePath")
    casing_added = _add_breps(doc, casing_shell, "GenerateInternalWire_Casing")
    node_added = _add_breps(doc, node_union, "GenerateInternalWire_Nodes")
    doc.Views.Redraw()
    return path_added and casing_added and node_added


def run_generate_internal_wire() -> Rhino.Commands.Result:
    doc = _active_doc()
    tolerance = doc.ModelAbsoluteTolerance
    min_node_gap = _mm_to_model(doc, MIN_NODE_GAP_MM)

    objref = _select_target_geometry()
    if objref is None:
        return Rhino.Commands.Result.Cancel

    mesh = _mesh_from_objref(objref, tolerance)
    if mesh is None:
        Rhino.RhinoApp.WriteLine(
            "The selected object must be a closed mesh or a solid that can be converted to a closed mesh."
        )
        return Rhino.Commands.Result.Failure

    options = _prompt_for_options(mesh, doc)
    if options is None:
        return Rhino.Commands.Result.Cancel

    if (options.node_radius * 2.0) + tolerance < _mm_to_model(doc, MIN_NODE_DIAMETER_MM):
        Rhino.RhinoApp.WriteLine("NodeSize must be at least 0.5 mm.")
        return Rhino.Commands.Result.Failure

    if (options.wire_radius * 2.0) + tolerance < _mm_to_model(doc, MIN_WIRE_DIAMETER_MM):
        Rhino.RhinoApp.WriteLine("WireRadius must define a conductive path diameter of at least 0.5 mm.")
        return Rhino.Commands.Result.Failure

    nodes = _collect_nodes(mesh, options.node_radius, tolerance, min_node_gap)
    if nodes is None:
        return Rhino.Commands.Result.Cancel

    Rhino.RhinoApp.WriteLine("Building routing grid...")
    routing_clearance = options.wire_radius + options.clearance
    valid_cells, grid = _build_valid_grid(mesh, options.step, routing_clearance, tolerance)
    if not valid_cells:
        Rhino.RhinoApp.WriteLine(
            "No valid routing cells were found. Increase VoxelSize or reduce Clearance or NodeSize."
        )
        return Rhino.Commands.Result.Failure

    node_centers = [node.center_point for node in nodes]
    node_cells: List[GridIndex] = []
    for node_center in node_centers:
        cell = _find_nearest_valid_cell(node_center, valid_cells, grid)
        if cell is None:
            Rhino.RhinoApp.WriteLine(
                "A node could not be mapped into the routing volume. Reduce Clearance, reduce NodeSize, or use a smaller VoxelSize."
            )
            return Rhino.Commands.Result.Failure
        node_cells.append(cell)

    spacing_radius = max(0, int(math.ceil(((options.wire_radius * 2.0) + min_node_gap) / options.step)))
    node_exemption_radius = max(
        1,
        int(math.ceil((options.node_radius + (options.wire_radius * 2.0) + min_node_gap) / options.step)),
    )

    try:
        segments = route_node_sequence(
            valid_cells=valid_cells,
            node_sequence=node_cells,
            penalty_radius=0,
            penalty_weight=options.step,
            blocked_radius=spacing_radius,
            blocked_exemption_radius=node_exemption_radius,
            allow_diagonals=True,
        )
    except RoutingError as error:
        Rhino.RhinoApp.WriteLine(str(error))
        Rhino.RhinoApp.WriteLine(
            "Try increasing VoxelSize or choosing nodes with more internal space between them."
        )
        return Rhino.Commands.Result.Failure

    polyline_points = _segments_to_polyline_points(node_centers, segments, grid, tolerance)
    if not _add_output_geometry(
        doc,
        polyline_points,
        nodes,
        options.wire_radius,
        options.node_radius,
        options.clearance,
    ):
        Rhino.RhinoApp.WriteLine("Failed to add the conductive path, casing, or node solids to the document.")
        return Rhino.Commands.Result.Failure

    Rhino.RhinoApp.WriteLine(
        "Generated {} conductive node solid(s), one conductive path solid, and one casing solid.".format(
            len(nodes)
        )
    )
    Rhino.RhinoApp.WriteLine(
        "NodeSize is the node diameter. Clearance is the casing thickness around the conductive path. WireRadius defines the conductive path radius."
    )
    return Rhino.Commands.Result.Success