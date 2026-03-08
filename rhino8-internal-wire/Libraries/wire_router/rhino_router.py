from __future__ import annotations

import math
from typing import Iterable, List, Optional, Sequence, Set, Tuple

import Rhino
import Rhino.DocObjects as rd
import Rhino.Geometry as rg
import Rhino.Input as ri
import Rhino.Input.Custom as ric
import System
import scriptcontext as sc

from .core import GridIndex, GridSpec, RoutingError, index_to_point, route_node_sequence


MAX_ESTIMATED_CELLS = 180000


def _active_doc() -> Rhino.RhinoDoc:
    if "__rhino_doc__" in globals() and __rhino_doc__ is not None:
        return __rhino_doc__
    return sc.doc


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


def _prompt_for_options(mesh: rg.Mesh, doc: Rhino.RhinoDoc) -> Optional[Tuple[float, float, float]]:
    bbox = mesh.GetBoundingBox(True)
    diagonal = bbox.Diagonal.Length
    minimum = max(doc.ModelAbsoluteTolerance * 2.0, diagonal / 1000.0)
    default_step = max(minimum, diagonal / 55.0)
    default_clearance = max(doc.ModelAbsoluteTolerance, default_step * 0.75)
    default_radius = max(doc.ModelAbsoluteTolerance, default_step * 0.35)

    get_option = ric.GetOption()
    get_option.SetCommandPrompt("Set routing options. Press Enter to continue")
    get_option.AcceptNothing(True)
    get_option.AcceptEnterWhenDone(True)

    voxel_option = ric.OptionDouble(default_step, minimum, max(default_step * 20.0, minimum * 2.0))
    clearance_option = ric.OptionDouble(default_clearance, 0.0, max(diagonal / 2.0, default_clearance + minimum))
    radius_option = ric.OptionDouble(default_radius, doc.ModelAbsoluteTolerance, max(diagonal / 2.0, default_radius + minimum))

    get_option.AddOptionDouble("VoxelSize", voxel_option)
    get_option.AddOptionDouble("Clearance", clearance_option)
    get_option.AddOptionDouble("WireRadius", radius_option)

    while True:
        result = get_option.Get()
        if result == ri.GetResult.Option:
            continue
        if result == ri.GetResult.Nothing:
            step = voxel_option.CurrentValue
            clearance = clearance_option.CurrentValue
            wire_radius = radius_option.CurrentValue
            estimated = _estimate_grid_cells(bbox, step)
            if estimated > MAX_ESTIMATED_CELLS:
                adjusted_step = step * ((estimated / float(MAX_ESTIMATED_CELLS)) ** (1.0 / 3.0))
                Rhino.RhinoApp.WriteLine(
                    "VoxelSize increased from {:.4f} to {:.4f} to keep the routing grid manageable.".format(
                        step, adjusted_step
                    )
                )
                step = adjusted_step
            return step, clearance, wire_radius
        return None


def _collect_node_points(mesh: rg.Mesh) -> Optional[List[rg.Point3d]]:
    points: List[rg.Point3d] = []

    while True:
        gp = ric.GetPoint()
        gp.SetCommandPrompt(
            "Pick node point {}. Press Enter when done".format(len(points) + 1)
        )
        gp.Constrain(mesh, False)
        gp.AcceptNothing(len(points) >= 2)
        gp.AcceptEnterWhenDone(True)
        if points:
            gp.SetBasePoint(points[-1], False)
            gp.DrawLineFromPoint(points[-1], False)

        result = gp.Get()
        if result == ri.GetResult.Point:
            points.append(gp.Point())
            continue
        if result == ri.GetResult.Nothing and len(points) >= 2:
            return points
        if result == ri.GetResult.Cancel:
            return None
        Rhino.RhinoApp.WriteLine("Pick at least two node points.")


def _mesh_distance(mesh: rg.Mesh, point: rg.Point3d) -> float:
    mesh_point = mesh.ClosestMeshPoint(point, 0.0)
    if mesh_point is not None:
        return point.DistanceTo(mesh.PointAt(mesh_point))
    return point.DistanceTo(mesh.ClosestPoint(point))


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


def _add_output_geometry(
    doc: Rhino.RhinoDoc,
    polyline_points: Sequence[rg.Point3d],
    wire_radius: float,
) -> bool:
    if len(polyline_points) < 2:
        return False

    curve = rg.PolylineCurve(polyline_points)
    if not curve.IsValid:
        return False

    curve_attributes = rd.ObjectAttributes()
    curve_attributes.Name = "GenerateInternalWire_Centerline"
    curve_id = doc.Objects.AddCurve(curve, curve_attributes)
    if curve_id == System.Guid.Empty:
        return False

    pipes = rg.Brep.CreatePipe(
        curve,
        wire_radius,
        False,
        rg.PipeCapMode.Round,
        True,
        doc.ModelAbsoluteTolerance,
        doc.ModelAngleToleranceRadians,
    )
    if pipes:
        pipe_attributes = rd.ObjectAttributes()
        pipe_attributes.Name = "GenerateInternalWire_ConductivePath"
        for pipe in pipes:
            doc.Objects.AddBrep(pipe, pipe_attributes)

    doc.Views.Redraw()
    return True


def run_generate_internal_wire() -> Rhino.Commands.Result:
    doc = _active_doc()
    tolerance = doc.ModelAbsoluteTolerance

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
    step, clearance, wire_radius = options

    node_points = _collect_node_points(mesh)
    if node_points is None:
        return Rhino.Commands.Result.Cancel

    Rhino.RhinoApp.WriteLine("Building routing grid...")
    valid_cells, grid = _build_valid_grid(mesh, step, clearance, tolerance)
    if not valid_cells:
        Rhino.RhinoApp.WriteLine(
            "No valid routing cells were found. Increase VoxelSize or reduce Clearance."
        )
        return Rhino.Commands.Result.Failure

    node_cells: List[GridIndex] = []
    for point in node_points:
        cell = _find_nearest_valid_cell(point, valid_cells, grid)
        if cell is None:
            Rhino.RhinoApp.WriteLine(
                "A node could not be mapped into the routing volume. Reduce Clearance or use a smaller VoxelSize."
            )
            return Rhino.Commands.Result.Failure
        node_cells.append(cell)

    try:
        # Multi-node routes are a single continuous wire. Penalizing a dilated
        # neighborhood around earlier segments makes later segments fail or kink
        # badly in narrow volumes, so only apply a soft penalty to exact reused cells.
        segments = route_node_sequence(
            valid_cells=valid_cells,
            node_sequence=node_cells,
            penalty_radius=0,
            penalty_weight=step * 2.0,
            allow_diagonals=True,
        )
    except RoutingError as error:
        Rhino.RhinoApp.WriteLine(str(error))
        Rhino.RhinoApp.WriteLine(
            "Try increasing VoxelSize, reducing Clearance, or choosing nodes with more internal space between them."
        )
        return Rhino.Commands.Result.Failure

    polyline_points = _segments_to_polyline_points(node_points, segments, grid, tolerance)
    if not _add_output_geometry(doc, polyline_points, wire_radius):
        Rhino.RhinoApp.WriteLine("Failed to add the routed wire geometry to the document.")
        return Rhino.Commands.Result.Failure

    Rhino.RhinoApp.WriteLine(
        "Generated wire through {} nodes using {} routed segment(s).".format(
            len(node_points), len(segments)
        )
    )
    Rhino.RhinoApp.WriteLine(
        "The pipe named GenerateInternalWire_ConductivePath is the generated conductive-path volume; the selected solid is not modified."
    )
    return Rhino.Commands.Result.Success
