from __future__ import annotations

import heapq
import math
import time
from dataclasses import dataclass
from typing import Dict, Iterable, Iterator, List, Optional, Sequence, Set, Tuple

GridIndex = Tuple[int, int, int]
FloatPoint = Tuple[float, float, float]
RoominessMap = Dict[GridIndex, int]


class RoutingError(RuntimeError):
    """Raised when a valid route cannot be found."""


_DILATION_OFFSET_CACHE: Dict[int, Tuple[Tuple[int, int, int], ...]] = {}


@dataclass(frozen=True)
class GridSpec:
    origin: FloatPoint
    step: float


@dataclass(frozen=True)
class NodeOrderMetrics:
    order: Tuple[int, ...]
    total_path_length: float
    start_leg_length: float
    touch_leg_lengths: Tuple[float, ...]
    end_leg_length: float

    @property
    def min_touch_leg_length(self) -> float:
        if not self.touch_leg_lengths:
            return 0.0
        return min(self.touch_leg_lengths)


def index_to_point(index: GridIndex, grid: GridSpec) -> FloatPoint:
    return (
        grid.origin[0] + index[0] * grid.step,
        grid.origin[1] + index[1] * grid.step,
        grid.origin[2] + index[2] * grid.step,
    )


def _distance(a: GridIndex, b: GridIndex) -> float:
    dx = float(a[0] - b[0])
    dy = float(a[1] - b[1])
    dz = float(a[2] - b[2])
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def _movement_distance(a: GridIndex, b: GridIndex, allow_diagonals: bool) -> float:
    if allow_diagonals:
        return _distance(a, b)
    return float(abs(a[0] - b[0]) + abs(a[1] - b[1]) + abs(a[2] - b[2]))


def _dilation_offsets(radius: int) -> Tuple[Tuple[int, int, int], ...]:
    """Return cached offset tuples for the given Chebyshev radius."""
    if radius in _DILATION_OFFSET_CACHE:
        return _DILATION_OFFSET_CACHE[radius]
    offsets = tuple(
        (dx, dy, dz)
        for dx in range(-radius, radius + 1)
        for dy in range(-radius, radius + 1)
        for dz in range(-radius, radius + 1)
    )
    _DILATION_OFFSET_CACHE[radius] = offsets
    return offsets


def _local_roominess(valid_cells: Set[GridIndex], center: GridIndex, radius: int = 2) -> int:
    offsets = _dilation_offsets(radius)
    cx, cy, cz = center
    return sum(1 for dx, dy, dz in offsets if (cx + dx, cy + dy, cz + dz) in valid_cells)


def _build_roominess_map(valid_cells: Set[GridIndex], radius: int = 2) -> RoominessMap:
    return {
        cell: _local_roominess(valid_cells, cell, radius)
        for cell in valid_cells
    }


def _roominess(roominess_map: RoominessMap, valid_cells: Set[GridIndex], cell: GridIndex) -> int:
    if cell in roominess_map:
        return roominess_map[cell]
    return _local_roominess(valid_cells, cell)


def _bottleneck_threshold(roominess_map: RoominessMap) -> int:
    if not roominess_map:
        return 0
    values = sorted(roominess_map.values())
    percentile_index = max(0, min(len(values) - 1, int(len(values) * 0.2)))
    return values[percentile_index]


def _path_bottleneck_penalty(
    path: Sequence[GridIndex],
    roominess_map: RoominessMap,
    valid_cells: Set[GridIndex],
    threshold: int,
) -> int:
    if threshold <= 0:
        return 0
    return sum(
        max(0, threshold - _roominess(roominess_map, valid_cells, cell))
        for cell in path[1:-1]
    )


def _pipe_corridor_radius(roominess: int, bottleneck_threshold: int) -> int:
    if roominess <= bottleneck_threshold:
        return 1
    return min(6, max(2, 1 + ((roominess - bottleneck_threshold + 3) // 4)))


def _build_pipe_corridor(
    pipe_path: Sequence[GridIndex],
    valid_cells: Set[GridIndex],
    roominess_map: RoominessMap,
    bottleneck_threshold: int,
) -> Set[GridIndex]:
    corridor: Set[GridIndex] = set()
    for cell in pipe_path:
        radius = _pipe_corridor_radius(
            _roominess(roominess_map, valid_cells, cell),
            bottleneck_threshold,
        )
        corridor.update(candidate for candidate in dilate_cells({cell}, radius) if candidate in valid_cells)
    corridor.update(pipe_path)
    return corridor


def _axis_walk(
    start: GridIndex, goal: GridIndex, valid_cells: Set[GridIndex]
) -> Optional[List[GridIndex]]:
    """Walk from *start* to *goal* stepping one axis at a time.

    Returns cells **excluding** *start* but **including** *goal*, or ``None``
    if any intermediate cell is not in *valid_cells*.
    """
    result: List[GridIndex] = []
    current = [start[0], start[1], start[2]]
    target = [goal[0], goal[1], goal[2]]
    for axis in range(3):
        if current[axis] == target[axis]:
            continue
        step_dir = 1 if target[axis] > current[axis] else -1
        while current[axis] != target[axis]:
            current[axis] += step_dir
            cell = (current[0], current[1], current[2])
            if cell not in valid_cells:
                return None
            result.append(cell)
    return result


def _connected_component(cells: Set[GridIndex], seed: GridIndex) -> Set[GridIndex]:
    if seed not in cells:
        return set()
    component: Set[GridIndex] = set()
    queue: List[GridIndex] = [seed]
    seen: Set[GridIndex] = {seed}
    while queue:
        current = queue.pop()
        component.add(current)
        for offset, _ in _AXIAL_NEIGHBOR_OFFSETS:
            neighbor = (
                current[0] + offset[0],
                current[1] + offset[1],
                current[2] + offset[2],
            )
            if neighbor in cells and neighbor not in seen:
                seen.add(neighbor)
                queue.append(neighbor)
    return component


def _build_fill_corridor(
    pipe_path: Sequence[GridIndex],
    valid_cells: Set[GridIndex],
    target_length: float,
    self_avoid_radius: int,
    start: Optional[GridIndex] = None,
    goal: Optional[GridIndex] = None,
    roominess_map: Optional[RoominessMap] = None,
    bottleneck_threshold: int = 0,
    min_self_spacing: int = 0,
) -> Set[GridIndex]:
    """Build a corridor around the pipe wide enough for a serpentine fill."""
    # Use the same spacing as _generate_serpentine_fill so the radius
    # estimate is consistent with the actual row/layer layout.
    row_spacing = max(2, self_avoid_radius + 2, min_self_spacing)
    pipe_length = max(1.0, _path_length(pipe_path))
    needed_r_sq = target_length * row_spacing * row_spacing / (4.0 * pipe_length)
    # Use a generous multiplier (3.0) so the serpentine can exploit
    # available space even when multiple segments compete for room.
    radius = max(4, int(math.ceil(math.sqrt(max(1.0, needed_r_sq)) * 3.0)))

    pipe_xs = [p[0] for p in pipe_path]
    pipe_ys = [p[1] for p in pipe_path]
    pipe_zs = [p[2] for p in pipe_path]
    min_x = min(pipe_xs) - radius
    max_x = max(pipe_xs) + radius
    min_y = min(pipe_ys) - radius
    max_y = max(pipe_ys) + radius
    min_z = min(pipe_zs) - radius
    max_z = max(pipe_zs) + radius
    corridor = {
        cell for cell in valid_cells
        if min_x <= cell[0] <= max_x
        and min_y <= cell[1] <= max_y
        and min_z <= cell[2] <= max_z
    }

    # Prefer roomy regions (higher local free-space) while preserving
    # a guaranteed neighborhood around the pipe backbone.
    if roominess_map and bottleneck_threshold > 0 and corridor:
        roomy_cells = {
            cell for cell in corridor
            if _roominess(roominess_map, valid_cells, cell) >= bottleneck_threshold
        }
        pipe_neighborhood = {
            cell for cell in dilate_cells(set(pipe_path), max(2, row_spacing))
            if cell in corridor
        }
        preferred = roomy_cells | pipe_neighborhood
        if len(preferred) >= max(32, int(len(corridor) * 0.45)):
            corridor = preferred

    # Keep only the connected component between start and goal when possible.
    if start is not None and goal is not None and start in corridor and goal in corridor:
        connected = _connected_component(corridor, start)
        if goal in connected:
            corridor = connected

    # If the bounding-box corridor is too small relative to the
    # target, fall back to all valid cells so the serpentine can
    # fill whatever space remains in the grid.
    min_cells_needed = int(target_length * max(1.5, row_spacing * 0.5))
    if len(corridor) < min_cells_needed:
        corridor = set(valid_cells)

    if start is not None and goal is not None and start in corridor and goal in corridor:
        connected = _connected_component(corridor, start)
        if goal in connected:
            corridor = connected

    return corridor


def _generate_serpentine_fill(
    corridor: Set[GridIndex],
    valid_cells: Set[GridIndex],
    start: GridIndex,
    goal: GridIndex,
    target_length: float,
    self_avoid_radius: int,
    min_self_spacing: int = 0,
) -> Optional[List[GridIndex]]:
    """Fill *corridor* with a dense serpentine path to reach *target_length*.

    Directly inspired by ``trace_filling`` from the 3dp-singlewire-sensing
    research paper: sweep layer-by-layer, row-by-row, alternating direction,
    connected by axis-aligned transitions between rows and layers.
    """
    if len(corridor) < 4:
        return None

    # +2 gives one full cell of clearance beyond the blocked radius,
    # which prevents pipe geometry at bends from overlapping adjacent rows.
    row_spacing = max(2, self_avoid_radius + 2, min_self_spacing)
    layer_spacing = max(2, self_avoid_radius + 2, min_self_spacing)

    # Choose sweep axes: sweep along the longest corridor dimension,
    # layer through the shortest, rows in the middle.
    coords = list(zip(*corridor))  # ([x …], [y …], [z …])
    axis_ranges = []
    for axis in range(3):
        vals = coords[axis]
        axis_ranges.append((max(vals) - min(vals), axis))
    axis_ranges.sort(key=lambda r: r[0])
    layer_axis = axis_ranges[0][1]  # thinnest
    row_axis = axis_ranges[1][1]
    sweep_axis = axis_ranges[2][1]  # longest

    # Group corridor cells by (layer, row) → list of sweep values.
    layer_rows: Dict[int, Dict[int, List[int]]] = {}
    for cell in corridor:
        lv = cell[layer_axis]
        rv = cell[row_axis]
        sv = cell[sweep_axis]
        layer_rows.setdefault(lv, {}).setdefault(rv, []).append(sv)

    # Select layers with spacing, ordered from start toward goal.
    all_layers = sorted(layer_rows.keys())
    if start[layer_axis] > goal[layer_axis]:
        all_layers = list(reversed(all_layers))
    selected_layers: List[int] = []
    for lv in all_layers:
        if not selected_layers or abs(lv - selected_layers[-1]) >= layer_spacing:
            selected_layers.append(lv)

    # Build ordered cell sequence (the abstract serpentine schedule).
    ordered: List[GridIndex] = []
    flip_row = False
    flip_sweep = False

    for lv in selected_layers:
        rows = layer_rows[lv]
        row_vals = sorted(rows.keys(), reverse=flip_row)

        selected_rv: List[int] = []
        for rv in row_vals:
            if not selected_rv or abs(rv - selected_rv[-1]) >= row_spacing:
                selected_rv.append(rv)

        for rv in selected_rv:
            svs = sorted(rows[rv], reverse=flip_sweep)
            flip_sweep = not flip_sweep
            for sv in svs:
                cell_list = [0, 0, 0]
                cell_list[layer_axis] = lv
                cell_list[row_axis] = rv
                cell_list[sweep_axis] = sv
                ordered.append((cell_list[0], cell_list[1], cell_list[2]))

        flip_row = not flip_row

    if not ordered:
        return None

    # Exclude cells within self_avoid_radius of start/goal so the
    # prefix/suffix connectors do not create close-approach violations
    # against the serpentine body.
    endpoint_buffer = dilate_cells({start, goal}, max(0, self_avoid_radius))
    endpoint_buffer.discard(start)
    endpoint_buffer.discard(goal)
    ordered = [c for c in ordered if c not in endpoint_buffer]

    if not ordered:
        return None

    # Choose the orientation (flip_row / flip_sweep combination) that
    # places ordered[0] closest to *start*, keeping the prefix short.
    # The ordered list was generated with a specific flip state but we
    # can cheaply try the three other permutations.
    def _reorder(try_flip_row: bool, try_flip_sweep: bool) -> List[GridIndex]:
        buf: List[GridIndex] = []
        fr = try_flip_row
        fs = try_flip_sweep
        for lv in selected_layers:
            rows = layer_rows[lv]
            rv_all = sorted(rows.keys(), reverse=fr)
            sel: List[int] = []
            for rv in rv_all:
                if not sel or abs(rv - sel[-1]) >= row_spacing:
                    sel.append(rv)
            for rv in sel:
                svs = sorted(rows[rv], reverse=fs)
                fs = not fs
                for sv in svs:
                    cl = [0, 0, 0]
                    cl[layer_axis] = lv
                    cl[row_axis] = rv
                    cl[sweep_axis] = sv
                    buf.append((cl[0], cl[1], cl[2]))
            fr = not fr
        return [c for c in buf if c not in endpoint_buffer]

    best_ordered = ordered
    best_dist = _distance(start, ordered[0]) if ordered else float("inf")
    for fr in (False, True):
        for fs in (False, True):
            trial = _reorder(fr, fs)
            if trial:
                d = _distance(start, trial[0])
                if d < best_dist:
                    best_dist = d
                    best_ordered = trial
    ordered = best_ordered

    if not ordered:
        return None

    # ---- Build a connected grid path ----
    path: List[GridIndex] = []

    # Block only the *prefix* connector (start → first serpentine cell)
    # so the suffix A* doesn't route back through it.  Row-to-row
    # transitions are NOT blocked because the serpentine schedule's
    # row_spacing (>= self_avoid_radius + 2) already guarantees that
    # adjacent sweep rows have sufficient physical clearance.
    prefix_blocked: Set[GridIndex] = set()

    def _suffix_valid() -> Set[GridIndex]:
        """Valid cells for the suffix A*: exclude the prefix zone."""
        if not prefix_blocked:
            return valid_cells
        tv = valid_cells - prefix_blocked
        tv.add(start)
        tv.add(goal)
        return tv

    # Running length counter — avoids O(n²) from recomputing
    # _path_length(path) inside the loop.
    running_length = 0.0

    def _extend_and_track(new_cells: Sequence[GridIndex]) -> None:
        nonlocal running_length
        for cell in new_cells:
            if path:
                running_length += _distance(path[-1], cell)
            path.append(cell)

    # 1) Route from *start* to the first serpentine cell.
    if start == ordered[0]:
        path.append(start)
    else:
        try:
            prefix = astar_path(valid_cells, start, ordered[0])
            _extend_and_track(prefix)
            if self_avoid_radius > 0:
                prefix_blocked = dilate_cells(set(prefix), self_avoid_radius)
        except RoutingError:
            return None

    # 2) Walk through the serpentine schedule.
    for cell in ordered:
        if cell == path[-1]:
            continue

        snapshot = len(path)
        snapshot_length = running_length
        last = path[-1]
        md = abs(cell[0] - last[0]) + abs(cell[1] - last[1]) + abs(cell[2] - last[2])

        if md == 1:
            running_length += 1.0
            path.append(cell)
        else:
            walk = _axis_walk(last, cell, valid_cells)
            if walk is not None:
                _extend_and_track(walk)
            else:
                try:
                    micro = astar_path(valid_cells, last, cell)
                    _extend_and_track(micro[1:])
                except RoutingError:
                    # Rollback any partial approach toward the unreachable
                    # cell so the path is not stranded at a dead-end.
                    del path[snapshot:]
                    running_length = snapshot_length
                    continue

        # Early stop once the serpentine has generated enough length.
        est_remaining = _distance(path[-1], goal)
        if running_length + est_remaining >= target_length:
            break

    # 3) Route from the serpentine exit to *goal*.
    if path[-1] != goal:
        # Avoid the serpentine body so the suffix doesn't create
        # reversals that _remove_path_reversals would collapse.
        body = set(path)
        suffix_cells = valid_cells - body
        suffix_cells -= prefix_blocked
        suffix_cells.add(path[-1])
        suffix_cells.add(goal)
        try:
            suffix = astar_path(suffix_cells, path[-1], goal)
        except RoutingError:
            # Fall back to the prefix-only filter if the body-avoiding
            # route is impossible (e.g. dense fills).
            try:
                suffix = astar_path(_suffix_valid(), path[-1], goal)
            except RoutingError:
                return None
        path.extend(suffix[1:])

    # Remove consecutive duplicates introduced by transition overlaps.
    deduped: List[GridIndex] = [path[0]]
    for cell in path[1:]:
        if cell != deduped[-1]:
            deduped.append(cell)

    deduped = _remove_path_reversals(deduped)
    deduped = _repair_path_continuity(deduped, valid_cells)
    deduped = _remove_path_reversals(deduped)

    # Serpentine spacing correctness is guaranteed by row_spacing and
    # layer_spacing (both >= self_avoid_radius + 2).  Do NOT apply
    # _has_nonlocal_close_approach here — the tight U-turns between
    # sweep rows are intentional and the geometry clearance is handled
    # by the schedule's spacing parameters, not by Chebyshev-distance
    # self-avoidance (which is designed for waypoint/growth paths).

    return deduped


def _remove_path_reversals(path: List[GridIndex]) -> List[GridIndex]:
    """Remove loops where the path revisits an earlier cell.

    When cell ``path[j]`` equals ``path[i]`` (with ``j > i``), the sub-path
    ``path[i+1 : j]`` is a loop/spur.  We keep the first visit and skip to
    the second, effectively short-circuiting the reversal.
    """
    if len(path) <= 2:
        return list(path)

    cleaned: List[GridIndex] = []
    seen: Dict[GridIndex, int] = {}
    index = 0
    while index < len(path):
        cell = path[index]
        if cell in seen:
            # Trim back to the first occurrence and continue from here.
            trim_to = seen[cell]
            cleaned = cleaned[: trim_to + 1]
            # Rebuild the lookup to match the trimmed list.
            seen = {c: i for i, c in enumerate(cleaned)}
        else:
            cleaned.append(cell)
            seen[cell] = len(cleaned) - 1
        index += 1
    return cleaned


def _repair_path_continuity(
    path: List[GridIndex],
    valid_cells: Set[GridIndex],
) -> List[GridIndex]:
    """Ensure every consecutive cell pair is grid-adjacent (Manhattan dist 1).

    Non-adjacent gaps are repaired with a short A* micro-path.  If a gap
    cannot be bridged the segment is kept as-is (the caller may still use
    the longer straight connector).
    """
    if len(path) < 2:
        return list(path)

    repaired: List[GridIndex] = [path[0]]
    for cell in path[1:]:
        prev = repaired[-1]
        md = abs(cell[0] - prev[0]) + abs(cell[1] - prev[1]) + abs(cell[2] - prev[2])
        if md <= 1:
            if cell != prev:
                repaired.append(cell)
            continue
        # Non-adjacent — try to bridge.
        walk = _axis_walk(prev, cell, valid_cells)
        if walk is not None:
            repaired.extend(walk)
        else:
            try:
                bridge = astar_path(valid_cells, prev, cell)
                repaired.extend(bridge[1:])
            except RoutingError:
                repaired.append(cell)
    return repaired


def _path_length(path: Sequence[GridIndex]) -> float:
    if len(path) < 2:
        return 0.0
    return sum(_distance(start, end) for start, end in zip(path[:-1], path[1:]))


def _path_vertical_span(path: Sequence[GridIndex]) -> int:
    if not path:
        return 0
    z_values = [point[2] for point in path]
    return max(z_values) - min(z_values)


def _path_layer_count(path: Sequence[GridIndex]) -> int:
    return len({point[2] for point in path})


def _path_xy_footprint(path: Sequence[GridIndex]) -> int:
    if not path:
        return 0
    x_values = [point[0] for point in path]
    y_values = [point[1] for point in path]
    return (max(x_values) - min(x_values) + 1) * (max(y_values) - min(y_values) + 1)


def _offset_index(index: GridIndex, offset: GridIndex, scale: int = 1) -> GridIndex:
    return (
        index[0] + (offset[0] * scale),
        index[1] + (offset[1] * scale),
        index[2] + (offset[2] * scale),
    )


def _negate_offset(offset: GridIndex) -> GridIndex:
    return (-offset[0], -offset[1], -offset[2])


def _nonzero_axis(offset: GridIndex) -> int:
    for axis, value in enumerate(offset):
        if value != 0:
            return axis
    raise ValueError("Offset has no non-zero axis.")


def _orthogonal_offsets(step_offset: GridIndex) -> List[GridIndex]:
    axis = _nonzero_axis(step_offset)
    offsets: List[GridIndex] = []
    for orthogonal_axis in range(3):
        if orthogonal_axis == axis:
            continue
        positive = [0, 0, 0]
        positive[orthogonal_axis] = 1
        offsets.append((positive[0], positive[1], positive[2]))
        offsets.append((-positive[0], -positive[1], -positive[2]))
    return offsets


def _segment_step_offset(start: GridIndex, goal: GridIndex) -> Optional[GridIndex]:
    dx = goal[0] - start[0]
    dy = goal[1] - start[1]
    dz = goal[2] - start[2]
    if abs(dx) + abs(dy) + abs(dz) != 1:
        return None
    return (dx, dy, dz)


def _has_nonlocal_close_approach(
    path: Sequence[GridIndex],
    radius: int,
    local_window: int = 3,
) -> bool:
    if radius <= 0:
        return len(set(path)) != len(path)

    # Spatial-dict approach: O(n * r^3) instead of O(n^2).
    offsets = _dilation_offsets(radius)
    earliest: Dict[GridIndex, int] = {}
    for i, cell in enumerate(path):
        for dx, dy, dz in offsets:
            nb = (cell[0] + dx, cell[1] + dy, cell[2] + dz)
            if nb in earliest and i - earliest[nb] > local_window:
                return True
        if cell not in earliest:
            earliest[cell] = i
    return False


def _cross_segment_near_approach(
    seg_a: Sequence[GridIndex],
    seg_b: Sequence[GridIndex],
    radius: int,
    shared_node: GridIndex,
    node_adjacency: int = 2,
) -> bool:
    """Return True if *seg_b* comes within *radius* cells of *seg_a*
    (Chebyshev distance), ignoring cells within *node_adjacency* of the
    *shared_node* where the two segments meet.

    This catches the case where two consecutive segments approach or leave
    a node from the same direction, overlapping just outside the node
    exemption zone.
    """
    if radius <= 0:
        return False

    node_zone = dilate_cells({shared_node}, node_adjacency)
    a_cells = {cell for cell in seg_a if cell not in node_zone}
    if not a_cells:
        return False
    blocked = dilate_cells(a_cells, radius)

    for cell in seg_b:
        if cell in node_zone:
            continue
        if cell in blocked:
            return True
    return False


def _approach_direction(
    path: Sequence[GridIndex],
    from_end: bool,
    depth: int = 3,
) -> Optional[GridIndex]:
    """Return a unit-step direction vector describing how *path* approaches
    the start (from_end=False) or end (from_end=True) of the path.

    Averages up to *depth* step directions then quantises to the dominant
    axis.  Returns ``None`` if the path is too short.
    """
    if len(path) < 2:
        return None

    if from_end:
        segment = list(reversed(path[-min(depth + 1, len(path)):]))
    else:
        segment = list(path[:min(depth + 1, len(path))])

    dx, dy, dz = 0, 0, 0
    for a, b in zip(segment[:-1], segment[1:]):
        dx += b[0] - a[0]
        dy += b[1] - a[1]
        dz += b[2] - a[2]

    # Quantise to the dominant axis (unit step).
    adx, ady, adz = abs(dx), abs(dy), abs(dz)
    dominant = max(adx, ady, adz)
    if dominant == 0:
        return None
    if adx == dominant:
        return (1 if dx > 0 else -1, 0, 0)
    if ady == dominant:
        return (0, 1 if dy > 0 else -1, 0)
    return (0, 0, 1 if dz > 0 else -1)


def _candidate_edge_detours(
    start: GridIndex,
    goal: GridIndex,
    max_primary_depth: int,
    max_secondary_depth: int,
) -> List[List[GridIndex]]:
    step_offset = _segment_step_offset(start, goal)
    if step_offset is None:
        return []

    candidates: List[List[GridIndex]] = []
    orthogonal = _orthogonal_offsets(step_offset)
    for primary_offset in orthogonal:
        for primary_depth in range(1, max_primary_depth + 1):
            simple: List[GridIndex] = [start]
            current = start
            for _ in range(primary_depth):
                current = _offset_index(current, primary_offset)
                simple.append(current)
            current = _offset_index(current, step_offset)
            simple.append(current)
            for _ in range(primary_depth):
                current = _offset_index(current, _negate_offset(primary_offset))
                simple.append(current)
            candidates.append(simple)

            for secondary_offset in orthogonal:
                if _nonzero_axis(secondary_offset) == _nonzero_axis(primary_offset):
                    continue
                for secondary_depth in range(1, max_secondary_depth + 1):
                    boxed: List[GridIndex] = [start]
                    current = start
                    for _ in range(primary_depth):
                        current = _offset_index(current, primary_offset)
                        boxed.append(current)
                    for _ in range(secondary_depth):
                        current = _offset_index(current, secondary_offset)
                        boxed.append(current)
                    current = _offset_index(current, step_offset)
                    boxed.append(current)
                    for _ in range(secondary_depth):
                        current = _offset_index(current, _negate_offset(secondary_offset))
                        boxed.append(current)
                    for _ in range(primary_depth):
                        current = _offset_index(current, _negate_offset(primary_offset))
                        boxed.append(current)
                    candidates.append(boxed)

    return candidates


def _is_valid_detour_candidate(
    candidate: Sequence[GridIndex],
    valid_cells: Set[GridIndex],
    occupied_cells: Set[GridIndex],
    self_avoid_radius: int,
    precomputed_blocked: Optional[Set[GridIndex]] = None,
) -> bool:
    if len(set(candidate)) != len(candidate):
        return False
    if any(cell not in valid_cells for cell in candidate):
        return False

    internal = set(candidate[1:-1])
    if internal & occupied_cells:
        return False

    if self_avoid_radius <= 0:
        return True

    blocked_cells = precomputed_blocked if precomputed_blocked is not None else dilate_cells(occupied_cells, self_avoid_radius)
    if internal & blocked_cells:
        return False

    return not _has_nonlocal_close_approach(candidate, self_avoid_radius)


def _grow_path_toward_target(
    path: Sequence[GridIndex],
    valid_cells: Set[GridIndex],
    target_length: float,
    self_avoid_radius: int,
    roominess_map: RoominessMap,
    bottleneck_threshold: int,
    growth_valid_cells: Optional[Set[GridIndex]] = None,
) -> List[GridIndex]:
    grown = list(path)
    if len(grown) < 2:
        return grown

    candidate_valid_cells = growth_valid_cells or valid_cells

    current_length = _path_length(grown)
    if current_length + 1e-9 >= target_length:
        return grown

    max_iterations = min(
        max(16, int(target_length - current_length)),
        max(16, len(valid_cells)),
    )
    for _ in range(max_iterations):
        if current_length + 1e-9 >= target_length:
            break

        edge_indices = sorted(
            range(len(grown) - 1),
            key=lambda index: min(
                _roominess(roominess_map, candidate_valid_cells, grown[index]),
                _roominess(roominess_map, candidate_valid_cells, grown[index + 1]),
            ),
            reverse=True,
        )[:40]

        best_replacement: Optional[List[GridIndex]] = None
        best_score: Optional[Tuple[float, bool, int, int, int, float]] = None
        for edge_index in edge_indices:
            start = grown[edge_index]
            goal = grown[edge_index + 1]
            if _segment_step_offset(start, goal) is None:
                continue

            local_exemption = dilate_cells({start, goal}, max(1, self_avoid_radius + 1))
            local_occupied = {cell for cell in grown if cell not in local_exemption}
            edge_blocked = dilate_cells(local_occupied, self_avoid_radius) if self_avoid_radius > 0 else set()

            for candidate in _candidate_edge_detours(
                start,
                goal,
                max_primary_depth=4,
                max_secondary_depth=3,
            ):
                if not _is_valid_detour_candidate(
                    candidate,
                    candidate_valid_cells,
                    local_occupied,
                    self_avoid_radius,
                    precomputed_blocked=edge_blocked,
                ):
                    continue

                expanded_path = grown[:edge_index] + candidate + grown[edge_index + 2 :]
                if _has_nonlocal_close_approach(expanded_path, self_avoid_radius):
                    continue

                expanded_length = _path_length(expanded_path)
                shortfall = max(0.0, target_length - expanded_length)
                overshoot = max(0.0, expanded_length - target_length)
                score = (
                    shortfall,
                    overshoot,
                    _path_bottleneck_penalty(
                        expanded_path,
                        roominess_map,
                        valid_cells,
                        bottleneck_threshold,
                    ),
                    _path_vertical_span(expanded_path),
                    _path_layer_count(expanded_path),
                    _path_xy_footprint(expanded_path),
                    -expanded_length,
                )
                if best_score is None or score < best_score:
                    best_score = score
                    best_replacement = expanded_path

        if best_replacement is None:
            break

        grown = best_replacement
        current_length = _path_length(grown)

    return grown


def _point_to_segment_distance(point: GridIndex, start: GridIndex, goal: GridIndex) -> float:
    px, py, pz = (float(value) for value in point)
    sx, sy, sz = (float(value) for value in start)
    gx, gy, gz = (float(value) for value in goal)

    vx = gx - sx
    vy = gy - sy
    vz = gz - sz
    wx = px - sx
    wy = py - sy
    wz = pz - sz

    length_sq = (vx * vx) + (vy * vy) + (vz * vz)
    if length_sq <= 1e-9:
        return _distance(point, start)

    projection = ((wx * vx) + (wy * vy) + (wz * vz)) / length_sq
    projection = max(0.0, min(1.0, projection))
    closest_x = sx + (vx * projection)
    closest_y = sy + (vy * projection)
    closest_z = sz + (vz * projection)
    dx = px - closest_x
    dy = py - closest_y
    dz = pz - closest_z
    return math.sqrt((dx * dx) + (dy * dy) + (dz * dz))


def _is_better_target_path(
    candidate_length: float,
    existing_length: float,
    target_length: float,
) -> bool:
    candidate_error = abs(candidate_length - target_length)
    existing_error = abs(existing_length - target_length)
    if candidate_error + 1e-9 < existing_error:
        return True
    if existing_error + 1e-9 < candidate_error:
        return False
    return candidate_length > existing_length + 1e-9


def _select_detour_waypoints(
    valid_cells: Set[GridIndex],
    start: GridIndex,
    goal: GridIndex,
    direct_length: float,
    target_length: float,
    limit: int,
    allow_diagonals: bool,
    roominess_map: RoominessMap,
    bottleneck_threshold: int,
) -> List[GridIndex]:
    ranked: List[Tuple[Tuple[float, float, float, float, float], GridIndex]] = []
    for candidate in valid_cells:
        if candidate == start or candidate == goal:
            continue

        start_distance = _movement_distance(start, candidate, allow_diagonals)
        goal_distance = _movement_distance(candidate, goal, allow_diagonals)
        if min(start_distance, goal_distance) < 2.0:
            continue

        total_distance = start_distance + goal_distance
        if total_distance <= direct_length + 2.0:
            continue

        line_distance = _point_to_segment_distance(candidate, start, goal)
        roominess = _roominess(roominess_map, valid_cells, candidate)
        bottleneck_penalty = max(0, bottleneck_threshold - roominess)
        midplane_offset = abs((candidate[2] * 2) - (start[2] + goal[2]))
        score = (
            abs(total_distance - target_length),
            bottleneck_penalty,
            -roominess,
            -total_distance,
            midplane_offset,
            -line_distance,
            -min(start_distance, goal_distance),
        )
        ranked.append((score, candidate))

    ranked.sort(key=lambda item: item[0])
    if len(ranked) <= limit:
        return [candidate for _, candidate in ranked]

    selected: List[GridIndex] = []
    selected_set: Set[GridIndex] = set()
    primary_count = max(4, limit // 3)
    for _, candidate in ranked[:primary_count]:
        selected.append(candidate)
        selected_set.add(candidate)

    pool = [candidate for _, candidate in ranked[: min(len(ranked), limit * 8)]]
    while len(selected) < limit:
        best_candidate: Optional[GridIndex] = None
        best_score: Optional[Tuple[float, float, float, float]] = None
        for pool_index, candidate in enumerate(pool):
            if candidate in selected_set:
                continue
            diversity = min(_distance(candidate, existing) for existing in selected)
            line_distance = _point_to_segment_distance(candidate, start, goal)
            roominess = _roominess(roominess_map, valid_cells, candidate)
            bottleneck_penalty = max(0, bottleneck_threshold - roominess)
            score = (-bottleneck_penalty, roominess, diversity, line_distance, -float(pool_index))
            if best_score is None or score > best_score:
                best_score = score
                best_candidate = candidate
        if best_candidate is None:
            break
        selected.append(best_candidate)
        selected_set.add(best_candidate)

    return selected


def _route_through_waypoints(
    valid_cells: Set[GridIndex],
    waypoint_sequence: Sequence[GridIndex],
    penalty_cells: Set[GridIndex],
    penalty_weight: float,
    allow_diagonals: bool,
    self_avoid_radius: int,
    vertical_move_penalty: float,
    boundary_penalty_cells: Optional[Set[GridIndex]] = None,
    boundary_penalty_weight: float = 0.0,
) -> List[GridIndex]:
    combined: List[GridIndex] = []
    consumed_cells: Set[GridIndex] = set()
    full_sequence = list(waypoint_sequence)

    for segment_index, (start, goal) in enumerate(zip(full_sequence[:-1], full_sequence[1:])):
        segment_valid_cells = set(valid_cells)
        if consumed_cells:
            blocked_consumed = dilate_cells(consumed_cells, max(0, self_avoid_radius))
            segment_valid_cells.difference_update(blocked_consumed)
            segment_valid_cells.add(start)
            segment_valid_cells.add(goal)

        future_waypoints = set(full_sequence[segment_index + 2 :])
        if future_waypoints:
            segment_valid_cells.difference_update(future_waypoints)
            segment_valid_cells.add(start)
            segment_valid_cells.add(goal)

        segment_penalties = set(penalty_cells)
        segment_penalties.update(consumed_cells)
        segment_penalties.update(future_waypoints)
        segment_penalties.discard(start)
        segment_penalties.discard(goal)

        segment = astar_path(
            valid_cells=segment_valid_cells,
            start=start,
            goal=goal,
            penalty_cells=segment_penalties,
            penalty_weight=penalty_weight,
            allow_diagonals=allow_diagonals,
            vertical_move_penalty=vertical_move_penalty,
            boundary_penalty_cells=boundary_penalty_cells,
            boundary_penalty_weight=boundary_penalty_weight,
        )

        if combined:
            combined.extend(segment[1:])
        else:
            combined.extend(segment)
        consumed_cells.update(segment[1:-1])

    combined = _remove_path_reversals(combined)
    return combined


def _route_segment_with_target_length(
    valid_cells: Set[GridIndex],
    start: GridIndex,
    goal: GridIndex,
    penalty_cells: Set[GridIndex],
    penalty_weight: float,
    allow_diagonals: bool,
    target_length: Optional[float],
    vertical_move_penalty: float = 0.0,
) -> List[GridIndex]:
    direct_path = astar_path(
        valid_cells=valid_cells,
        start=start,
        goal=goal,
        penalty_cells=penalty_cells,
        penalty_weight=penalty_weight,
        allow_diagonals=allow_diagonals,
        vertical_move_penalty=vertical_move_penalty,
    )
    if target_length is None:
        return direct_path

    direct_length = _path_length(direct_path)
    if direct_length + 1e-9 >= target_length:
        return direct_path

    best_path = direct_path
    best_length = direct_length
    candidate_waypoints = _select_detour_waypoints(
        valid_cells=valid_cells,
        start=start,
        goal=goal,
        direct_length=direct_length,
        target_length=target_length,
        limit=18,
        allow_diagonals=allow_diagonals,
    )

    for waypoint in candidate_waypoints:
        try:
            candidate_path = _route_through_waypoints(
                valid_cells=valid_cells,
                waypoint_sequence=(start, waypoint, goal),
                penalty_cells=penalty_cells,
                penalty_weight=penalty_weight,
                allow_diagonals=allow_diagonals,
                self_avoid_radius=0,
                vertical_move_penalty=vertical_move_penalty,
            )
        except RoutingError:
            continue

        candidate_length = _path_length(candidate_path)
        if _is_better_target_path(candidate_length, best_length, target_length):
            best_path = candidate_path
            best_length = candidate_length

    if best_length + 1e-9 >= target_length:
        return best_path

    pair_candidates = candidate_waypoints[: min(6, len(candidate_waypoints))]
    for first_waypoint in pair_candidates:
        for second_waypoint in pair_candidates:
            if first_waypoint == second_waypoint:
                continue
            try:
                candidate_path = _route_through_waypoints(
                    valid_cells=valid_cells,
                    waypoint_sequence=(start, first_waypoint, second_waypoint, goal),
                    penalty_cells=penalty_cells,
                    penalty_weight=penalty_weight,
                    allow_diagonals=allow_diagonals,
                    self_avoid_radius=0,
                    vertical_move_penalty=vertical_move_penalty,
                )
            except RoutingError:
                continue

            candidate_length = _path_length(candidate_path)
            if _is_better_target_path(candidate_length, best_length, target_length):
                best_path = candidate_path
                best_length = candidate_length

    return best_path


def _dedupe_paths(paths: Sequence[Sequence[GridIndex]]) -> List[List[GridIndex]]:
    unique: List[List[GridIndex]] = []
    seen: Set[Tuple[GridIndex, ...]] = set()
    for path in paths:
        key = tuple(path)
        if key in seen:
            continue
        seen.add(key)
        unique.append(list(path))
    return unique


def _sort_candidate_paths(
    paths: Sequence[Sequence[GridIndex]],
    target_length: Optional[float],
    roominess_map: Optional[RoominessMap] = None,
    valid_cells: Optional[Set[GridIndex]] = None,
    bottleneck_threshold: int = 0,
) -> List[List[GridIndex]]:
    if target_length is None:
        return sorted((list(path) for path in paths), key=_path_length)

    roominess_lookup = roominess_map or {}
    valid = valid_cells or set()

    def _sort_key(path: List[GridIndex]) -> Tuple[float, float, int, int, int, int, float]:
        length = _path_length(path)
        return (
            max(0.0, target_length - length),
            max(0.0, length - target_length),
            _path_bottleneck_penalty(path, roominess_lookup, valid, bottleneck_threshold),
            _path_vertical_span(path),
            _path_layer_count(path),
            _path_xy_footprint(path),
            -length,
        )

    return sorted(
        (list(path) for path in paths),
        key=_sort_key,
    )


def _estimate_waypoint_chain_length(
    start: GridIndex,
    waypoint_sequence: Sequence[GridIndex],
    goal: GridIndex,
    allow_diagonals: bool,
) -> float:
    points = [start] + list(waypoint_sequence) + [goal]
    return sum(
        _movement_distance(previous, current, allow_diagonals)
        for previous, current in zip(points[:-1], points[1:])
    )


def _waypoint_chain_spread(
    waypoint_sequence: Sequence[GridIndex],
    allow_diagonals: bool,
) -> float:
    if len(waypoint_sequence) < 2:
        return 0.0
    return min(
        _movement_distance(previous, current, allow_diagonals)
        for previous, current in zip(waypoint_sequence[:-1], waypoint_sequence[1:])
    )


def _segment_candidate_paths(
    valid_cells: Set[GridIndex],
    start: GridIndex,
    goal: GridIndex,
    penalty_cells: Set[GridIndex],
    penalty_weight: float,
    allow_diagonals: bool,
    target_length: Optional[float],
    self_avoid_radius: int = 0,
    max_candidates: int = 8,
    vertical_move_penalty: float = 0.0,
    roominess_map: Optional[RoominessMap] = None,
    bottleneck_threshold: Optional[int] = None,
    boundary_penalty_cells: Optional[Set[GridIndex]] = None,
    boundary_penalty_weight: float = 0.0,
    beam_width: int = 8,
    beam_max_waypoints: int = 3,
    deadline: Optional[float] = None,
    min_self_spacing: int = 0,
) -> List[List[GridIndex]]:
    if roominess_map is None:
        roominess_map = _build_roominess_map(valid_cells)
    if bottleneck_threshold is None:
        bottleneck_threshold = _bottleneck_threshold(roominess_map)
    target_first_vertical_penalty = 0.0 if target_length is not None else vertical_move_penalty

    pipe_path = astar_path(
        valid_cells=valid_cells,
        start=start,
        goal=goal,
        penalty_cells=penalty_cells,
        penalty_weight=penalty_weight,
        allow_diagonals=allow_diagonals,
        vertical_move_penalty=vertical_move_penalty,
        boundary_penalty_cells=boundary_penalty_cells,
        boundary_penalty_weight=boundary_penalty_weight,
    )
    candidate_valid_cells = valid_cells
    if target_length is not None and not allow_diagonals:
        candidate_valid_cells = _build_pipe_corridor(
            pipe_path,
            valid_cells,
            roominess_map,
            bottleneck_threshold,
        )

    direct_path = pipe_path

    candidates: List[List[GridIndex]] = [direct_path]
    if target_length is None:
        return candidates

    direct_length = _path_length(direct_path)

    # ---- Serpentine fill (primary strategy for large targets) ----
    # Inspired by trace_filling from 3dp-singlewire-sensing.
    if not allow_diagonals and target_length > direct_length * 1.5:
        fill_corridor = _build_fill_corridor(
            pipe_path,
            valid_cells,
            target_length,
            self_avoid_radius,
            start=start,
            goal=goal,
            roominess_map=roominess_map,
            bottleneck_threshold=bottleneck_threshold,
            min_self_spacing=min_self_spacing,
        )
        serpentine = _generate_serpentine_fill(
            fill_corridor,
            valid_cells,
            start,
            goal,
            target_length,
            self_avoid_radius,
            min_self_spacing=min_self_spacing,
        )
        if serpentine is not None:
            candidates.append(serpentine)
            serpentine_length = _path_length(serpentine)
            if serpentine_length >= target_length * 0.85:
                # Serpentine reached the target — skip the expensive beam search.
                unique = _dedupe_paths(candidates)
                return _sort_candidate_paths(
                    unique,
                    target_length,
                    roominess_map=roominess_map,
                    valid_cells=valid_cells,
                    bottleneck_threshold=bottleneck_threshold,
                )[:max_candidates]

    candidate_waypoints = _select_detour_waypoints(
        valid_cells=candidate_valid_cells,
        start=start,
        goal=goal,
        direct_length=direct_length,
        target_length=target_length,
        limit=12,
        allow_diagonals=allow_diagonals,
        roominess_map=roominess_map,
        bottleneck_threshold=bottleneck_threshold,
    )

    beam: List[Tuple[GridIndex, ...]] = [tuple()]
    for _ in range(beam_max_waypoints):
        expanded: List[Tuple[Tuple[float, float, int, int, int, float, float], Tuple[GridIndex, ...]]] = []
        for waypoint_sequence in beam:
            used = set(waypoint_sequence)
            for waypoint in candidate_waypoints:
                if waypoint in used:
                    continue
                next_sequence = waypoint_sequence + (waypoint,)
                estimated_length = _estimate_waypoint_chain_length(
                    start,
                    next_sequence,
                    goal,
                    allow_diagonals,
                )
                expanded.append(
                    (
                        (
                            max(0.0, target_length - estimated_length),
                            max(0.0, estimated_length - target_length),
                            _path_bottleneck_penalty(
                                (start,) + next_sequence + (goal,),
                                roominess_map,
                                valid_cells,
                                bottleneck_threshold,
                            ),
                            _path_vertical_span((start,) + next_sequence + (goal,)),
                            _path_layer_count((start,) + next_sequence + (goal,)),
                            _path_xy_footprint((start,) + next_sequence + (goal,)),
                            -_waypoint_chain_spread(next_sequence, allow_diagonals),
                            -estimated_length,
                        ),
                        next_sequence,
                    )
                )

        if not expanded:
            break

        expanded.sort(key=lambda item: item[0])
        beam = [sequence for _, sequence in expanded[:beam_width]]
        for waypoint_sequence in beam:
            if deadline is not None and time.monotonic() > deadline:
                break
            try:
                candidates.append(
                    _route_through_waypoints(
                        valid_cells=candidate_valid_cells,
                        waypoint_sequence=(start,) + waypoint_sequence + (goal,),
                        penalty_cells=penalty_cells,
                        penalty_weight=penalty_weight,
                        allow_diagonals=allow_diagonals,
                        self_avoid_radius=self_avoid_radius,
                        vertical_move_penalty=target_first_vertical_penalty,
                        boundary_penalty_cells=boundary_penalty_cells,
                        boundary_penalty_weight=boundary_penalty_weight,
                    )
                )
            except RoutingError:
                continue
        if deadline is not None and time.monotonic() > deadline:
            break

    # Clean every candidate: remove self-overlapping loops, then dedupe.
    cleaned_candidates = [_remove_path_reversals(path) for path in candidates]
    unique_candidates = _dedupe_paths(cleaned_candidates)
    clearance_radius = max(0, self_avoid_radius, max(0, min_self_spacing - 1))
    if clearance_radius > 0:
        spaced_candidates = [
            path for path in unique_candidates
            if not _has_nonlocal_close_approach(path, clearance_radius)
        ]
        if spaced_candidates:
            unique_candidates = spaced_candidates

    if target_length is not None and not allow_diagonals:
        grown_candidates = [
            _grow_path_toward_target(
                path,
                valid_cells,
                target_length,
                max(0, self_avoid_radius),
                roominess_map,
                bottleneck_threshold,
                growth_valid_cells=candidate_valid_cells,
            )
            for path in unique_candidates
            if _path_length(path) < target_length * 0.95
        ]
        grown_cleaned = [_remove_path_reversals(path) for path in grown_candidates]
        unique_candidates = _dedupe_paths(unique_candidates + grown_cleaned)
    return _sort_candidate_paths(
        unique_candidates,
        target_length,
        roominess_map=roominess_map,
        valid_cells=valid_cells,
        bottleneck_threshold=bottleneck_threshold,
    )[:max_candidates]


_AXIAL_NEIGHBOR_OFFSETS: Tuple[Tuple[GridIndex, float], ...] = (
    ((1, 0, 0), 1.0),
    ((-1, 0, 0), 1.0),
    ((0, 1, 0), 1.0),
    ((0, -1, 0), 1.0),
    ((0, 0, 1), 1.0),
    ((0, 0, -1), 1.0),
)

_DIAGONAL_NEIGHBOR_OFFSETS: Tuple[Tuple[GridIndex, float], ...] = tuple(
    ((dx, dy, dz), math.sqrt(dx * dx + dy * dy + dz * dz))
    for dx in (-1, 0, 1)
    for dy in (-1, 0, 1)
    for dz in (-1, 0, 1)
    if not (dx == 0 and dy == 0 and dz == 0)
)


def _neighbor_offsets(allow_diagonals: bool) -> Tuple[Tuple[GridIndex, float], ...]:
    if allow_diagonals:
        return _DIAGONAL_NEIGHBOR_OFFSETS
    return _AXIAL_NEIGHBOR_OFFSETS


def _reconstruct_path(
    came_from: Dict[GridIndex, GridIndex], current: GridIndex
) -> List[GridIndex]:
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path


def astar_path(
    valid_cells: Set[GridIndex],
    start: GridIndex,
    goal: GridIndex,
    penalty_cells: Optional[Set[GridIndex]] = None,
    penalty_weight: float = 0.0,
    allow_diagonals: bool = True,
    vertical_move_penalty: float = 0.0,
    boundary_penalty_cells: Optional[Set[GridIndex]] = None,
    boundary_penalty_weight: float = 0.0,
) -> List[GridIndex]:
    if start not in valid_cells:
        raise RoutingError(f"Start cell {start} is not inside the valid routing grid.")
    if goal not in valid_cells:
        raise RoutingError(f"Goal cell {goal} is not inside the valid routing grid.")
    if start == goal:
        return [start]

    penalties = penalty_cells or set()
    boundary_penalties = boundary_penalty_cells or set()
    frontier: List[Tuple[float, int, GridIndex]] = []
    heapq.heappush(frontier, (0.0, 0, start))

    came_from: Dict[GridIndex, GridIndex] = {}
    g_score: Dict[GridIndex, float] = {start: 0.0}
    insertion_order = 1

    while frontier:
        _, _, current = heapq.heappop(frontier)
        if current == goal:
            return _reconstruct_path(came_from, current)

        current_cost = g_score[current]
        for offset, move_cost in _neighbor_offsets(allow_diagonals=allow_diagonals):
            neighbor = (
                current[0] + offset[0],
                current[1] + offset[1],
                current[2] + offset[2],
            )
            if neighbor not in valid_cells:
                continue

            tentative = current_cost + move_cost
            if vertical_move_penalty > 0.0 and offset[2] != 0:
                tentative += vertical_move_penalty * abs(offset[2])
            if neighbor in penalties:
                tentative += penalty_weight
            if boundary_penalties and neighbor in boundary_penalties:
                tentative += boundary_penalty_weight

            if tentative >= g_score.get(neighbor, float("inf")):
                continue

            came_from[neighbor] = current
            g_score[neighbor] = tentative
            priority = tentative + _distance(neighbor, goal)
            heapq.heappush(frontier, (priority, insertion_order, neighbor))
            insertion_order += 1

    raise RoutingError(f"No route found between {start} and {goal}.")


def dilate_cells(cells: Iterable[GridIndex], radius: int) -> Set[GridIndex]:
    cells = set(cells)
    if radius <= 0:
        return set(cells)

    offsets = _dilation_offsets(radius)
    dilated: Set[GridIndex] = set()
    for cx, cy, cz in cells:
        for dx, dy, dz in offsets:
            dilated.add((cx + dx, cy + dy, cz + dz))
    return dilated


def compress_index_path(path: Sequence[GridIndex]) -> List[GridIndex]:
    if len(path) <= 2:
        return list(path)

    compressed = [path[0]]
    previous_direction: Optional[GridIndex] = None

    for current, nxt in zip(path[:-1], path[1:]):
        direction = (
            max(-1, min(1, nxt[0] - current[0])),
            max(-1, min(1, nxt[1] - current[1])),
            max(-1, min(1, nxt[2] - current[2])),
        )
        if previous_direction is None:
            previous_direction = direction
            continue
        if direction != previous_direction:
            compressed.append(current)
        previous_direction = direction

    compressed.append(path[-1])
    return compressed


def evaluate_node_order(
    order: Sequence[int],
    start_distances: Sequence[float],
    end_distances: Sequence[float],
    pair_distances: Sequence[Sequence[float]],
) -> NodeOrderMetrics:
    ordered = tuple(order)
    if not ordered:
        return NodeOrderMetrics(
            order=(),
            total_path_length=0.0,
            start_leg_length=0.0,
            touch_leg_lengths=(),
            end_leg_length=0.0,
        )

    start_leg_length = start_distances[ordered[0]]
    total_path_length = start_leg_length
    touch_leg_lengths: List[float] = []

    for previous, current in zip(ordered[:-1], ordered[1:]):
        leg_length = pair_distances[previous][current]
        touch_leg_lengths.append(leg_length)
        total_path_length += leg_length

    end_leg_length = end_distances[ordered[-1]]
    total_path_length += end_leg_length
    return NodeOrderMetrics(
        order=ordered,
        total_path_length=total_path_length,
        start_leg_length=start_leg_length,
        touch_leg_lengths=tuple(touch_leg_lengths),
        end_leg_length=end_leg_length,
    )


def optimize_node_order_for_path(
    start_distances: Sequence[float],
    end_distances: Sequence[float],
    pair_distances: Sequence[Sequence[float]],
    max_exact_nodes: int = 10,
) -> Tuple[int, ...]:
    node_count = len(start_distances)
    if node_count == 0:
        return ()

    if node_count > max_exact_nodes:
        remaining = list(range(node_count))
        ordered: List[int] = []
        last_index: Optional[int] = None
        while remaining:
            if last_index is None:
                next_index = min(
                    remaining,
                    key=lambda index: start_distances[index] + (end_distances[index] * 0.1),
                )
            else:
                next_index = min(
                    remaining,
                    key=lambda index: pair_distances[last_index][index] + (end_distances[index] * 0.1),
                )
            ordered.append(next_index)
            remaining.remove(next_index)
            last_index = next_index
        return tuple(ordered)

    dp: Dict[Tuple[int, int], float] = {}
    parent: Dict[Tuple[int, int], Optional[int]] = {}

    for index in range(node_count):
        mask = 1 << index
        dp[(mask, index)] = start_distances[index]
        parent[(mask, index)] = None

    for mask in range(1, 1 << node_count):
        for last in range(node_count):
            state = (mask, last)
            if state not in dp:
                continue
            for nxt in range(node_count):
                if mask & (1 << nxt):
                    continue
                next_mask = mask | (1 << nxt)
                candidate = dp[state] + pair_distances[last][nxt]
                next_state = (next_mask, nxt)
                if candidate < dp.get(next_state, float("inf")):
                    dp[next_state] = candidate
                    parent[next_state] = last

    full_mask = (1 << node_count) - 1
    best_last = min(
        range(node_count),
        key=lambda index: dp[(full_mask, index)] + end_distances[index],
    )
    return _reconstruct_order(parent, full_mask, best_last)


def optimize_node_order_for_target_leg_length(
    start_distances: Sequence[float],
    end_distances: Sequence[float],
    pair_distances: Sequence[Sequence[float]],
    target_leg_length: float,
    max_exact_nodes: int = 10,
) -> Tuple[int, ...]:
    node_count = len(start_distances)
    if node_count == 0:
        return ()

    def leg_cost(distance: float) -> float:
        if distance < target_leg_length:
            shortfall = target_leg_length - distance
            return shortfall * 8.0

        overshoot = distance - target_leg_length
        return overshoot * 0.35

    if node_count > max_exact_nodes:
        remaining = list(range(node_count))
        ordered: List[int] = []
        last_index: Optional[int] = None
        while remaining:
            if last_index is None:
                next_index = max(
                    remaining,
                    key=lambda index: _average_pair_distance(index, remaining, pair_distances),
                )
            else:
                next_index = min(
                    remaining,
                    key=lambda index: leg_cost(pair_distances[last_index][index]),
                )
            ordered.append(next_index)
            remaining.remove(next_index)
            last_index = next_index
        return tuple(ordered)

    dp: Dict[Tuple[int, int], float] = {}
    parent: Dict[Tuple[int, int], Optional[int]] = {}

    for index in range(node_count):
        mask = 1 << index
        dp[(mask, index)] = 0.0
        parent[(mask, index)] = None

    for mask in range(1, 1 << node_count):
        for last in range(node_count):
            state = (mask, last)
            if state not in dp:
                continue
            for nxt in range(node_count):
                if mask & (1 << nxt):
                    continue
                next_mask = mask | (1 << nxt)
                candidate = dp[state] + leg_cost(pair_distances[last][nxt])
                next_state = (next_mask, nxt)
                if candidate < dp.get(next_state, float("inf")):
                    dp[next_state] = candidate
                    parent[next_state] = last

    full_mask = (1 << node_count) - 1
    best_last = min(
        range(node_count),
        key=lambda index: (dp[(full_mask, index)], start_distances[index] + end_distances[index]),
    )
    return _reconstruct_order(parent, full_mask, best_last)


def optimize_node_order_for_maximum_spacing(
    start_distances: Sequence[float],
    end_distances: Sequence[float],
    pair_distances: Sequence[Sequence[float]],
    max_exact_nodes: int = 10,
) -> Tuple[int, ...]:
    node_count = len(start_distances)
    if node_count == 0:
        return ()

    if node_count > max_exact_nodes:
        remaining = list(range(node_count))
        ordered: List[int] = []
        last_index: Optional[int] = None
        while remaining:
            if last_index is None:
                next_index = max(
                    remaining,
                    key=lambda index: (
                        _minimum_pair_distance(index, remaining, pair_distances),
                        _average_pair_distance(index, remaining, pair_distances),
                    ),
                )
            else:
                next_index = max(
                    remaining,
                    key=lambda index: pair_distances[last_index][index],
                )
            ordered.append(next_index)
            remaining.remove(next_index)
            last_index = next_index
        return tuple(ordered)

    score_dp: Dict[Tuple[int, int], Tuple[float, float]] = {}
    parent: Dict[Tuple[int, int], Optional[int]] = {}

    for index in range(node_count):
        mask = 1 << index
        score_dp[(mask, index)] = (float("inf"), 0.0)
        parent[(mask, index)] = None

    for mask in range(1, 1 << node_count):
        for last in range(node_count):
            state = (mask, last)
            if state not in score_dp:
                continue
            current_min_leg, current_total = score_dp[state]
            for nxt in range(node_count):
                if mask & (1 << nxt):
                    continue
                next_mask = mask | (1 << nxt)
                next_leg = pair_distances[last][nxt]
                candidate = (
                    min(current_min_leg, next_leg),
                    current_total + next_leg,
                )
                next_state = (next_mask, nxt)
                existing = score_dp.get(next_state)
                if existing is None or _is_better_spacing_score(candidate, existing):
                    score_dp[next_state] = candidate
                    parent[next_state] = last

    full_mask = (1 << node_count) - 1
    best_last = max(
        range(node_count),
        key=lambda index: (
            score_dp[(full_mask, index)][0],
            -score_dp[(full_mask, index)][1],
            -(start_distances[index] + end_distances[index]),
        ),
    )
    return _reconstruct_order(parent, full_mask, best_last)


def _average_pair_distance(
    index: int, remaining: Sequence[int], pair_distances: Sequence[Sequence[float]]
) -> float:
    others = [other for other in remaining if other != index]
    if not others:
        return 0.0
    return sum(pair_distances[index][other] for other in others) / float(len(others))


def _minimum_pair_distance(
    index: int, remaining: Sequence[int], pair_distances: Sequence[Sequence[float]]
) -> float:
    others = [other for other in remaining if other != index]
    if not others:
        return float("inf")
    return min(pair_distances[index][other] for other in others)


def _is_better_spacing_score(
    candidate: Tuple[float, float], existing: Tuple[float, float]
) -> bool:
    if candidate[0] > existing[0] + 1e-9:
        return True
    if existing[0] > candidate[0] + 1e-9:
        return False
    return candidate[1] + 1e-9 < existing[1]


def _reconstruct_order(
    parent: Dict[Tuple[int, int], Optional[int]], full_mask: int, best_last: int
) -> Tuple[int, ...]:
    order_indices: List[int] = []
    state = (full_mask, best_last)
    while True:
        mask, last = state
        order_indices.append(last)
        previous = parent[state]
        if previous is None:
            break
        state = (mask ^ (1 << last), previous)

    order_indices.reverse()
    return tuple(order_indices)


def route_node_sequence(
    valid_cells: Set[GridIndex],
    node_sequence: Sequence[GridIndex],
    segment_target_lengths: Optional[Sequence[Optional[float]]] = None,
    penalty_radius: int = 1,
    penalty_weight: float = 10.0,
    blocked_radius: int = 0,
    blocked_exemption_radius: int = 0,
    reserved_cells: Optional[Set[GridIndex]] = None,
    reserved_exemption_radius: int = 0,
    node_labels: Optional[Sequence[str]] = None,
    allow_diagonals: bool = True,
    vertical_move_penalty: float = 0.0,
    deadline: Optional[float] = None,
    min_self_spacing: int = 0,
) -> List[List[GridIndex]]:
    if len(node_sequence) < 2:
        raise RoutingError("At least two nodes are required to build a route.")

    if segment_target_lengths is not None and len(segment_target_lengths) != (len(node_sequence) - 1):
        raise RoutingError("Segment target lengths must match the routed segment count.")

    if node_labels is not None and len(node_labels) != len(node_sequence):
        raise RoutingError("Node labels must match the routed node sequence length.")

    segments: List[List[GridIndex]] = []
    reserved = reserved_cells or set()
    roominess_map = _build_roominess_map(valid_cells)
    bottleneck_threshold = _bottleneck_threshold(roominess_map)

    # Precompute node keepout zones: for each node, the set of cells
    # that paths NOT connecting to that node must avoid.  The radius
    # covers the physical node volume (reserved_exemption_radius) plus
    # the wire-to-node edge-to-edge clearance (blocked_radius).
    node_keepout_radius = reserved_exemption_radius + blocked_radius if blocked_radius > 0 else 0
    per_node_keepout: Dict[GridIndex, FrozenSet[GridIndex]] = {}
    if node_keepout_radius > 0 and len(node_sequence) > 2:
        for node in set(node_sequence):
            per_node_keepout[node] = frozenset(
                dilate_cells({node}, node_keepout_radius)
            )

    # Adaptive beam search: reduce beam width and waypoint depth for
    # high node counts to keep per-segment candidate generation fast.
    num_segments = len(node_sequence) - 1
    if num_segments > 10:
        seg_beam_width = 3
        seg_beam_max_waypoints = 1
    elif num_segments > 8:
        seg_beam_width = 4
        seg_beam_max_waypoints = 2
    else:
        seg_beam_width = 8
        seg_beam_max_waypoints = 3

    # Maximum number of segments to backtrack when a downstream segment
    # fails.  Deep backtracking is exponentially expensive and rarely
    # productive — failing the order early and trying the next one is
    # almost always a better strategy.
    max_backtrack_depth = 3

    # Pre-compute boundary-penalty cells: cells near the geometry boundary
    # (low roominess) get a soft A* cost penalty so paths prefer the
    # interior.  This leaves more room for serpentine fills and reduces
    # the chance of cross-segment overlaps near tight boundary regions.
    boundary_penalty_cells: Optional[Set[GridIndex]] = None
    if bottleneck_threshold > 0:
        boundary_penalty_cells = frozenset(
            cell for cell, roominess in roominess_map.items()
            if roominess < bottleneck_threshold
        )

    # Track the furthest segment reached so far (shared mutable state)
    # so that _search can enforce the backtracking-depth limit.
    frontier_reached = [0]

    def _search(
        segment_index: int,
        current_penalty_cells: Set[GridIndex],
        current_blocked_cells: Set[GridIndex],
        current_segments: List[List[GridIndex]],
        approach_directions: Dict[GridIndex, GridIndex],
    ) -> List[List[GridIndex]]:
        if segment_index >= len(node_sequence) - 1:
            return list(current_segments)

        # Enforce per-order time budget.
        if deadline is not None and time.monotonic() > deadline:
            raise RoutingError("Routing timed out for this node order.")

        # Enforce backtracking-depth limit.
        if segment_index > frontier_reached[0]:
            frontier_reached[0] = segment_index
        elif frontier_reached[0] - segment_index > max_backtrack_depth:
            raise RoutingError(
                "Backtracking depth exceeded at segment {}.".format(segment_index)
            )

        start = node_sequence[segment_index]
        goal = node_sequence[segment_index + 1]
        segment_valid_cells = set(valid_cells)
        if reserved:
            local_reserved = set(reserved)
            local_reserved.difference_update(
                dilate_cells({start, goal}, max(0, reserved_exemption_radius))
            )
            local_reserved.discard(start)
            local_reserved.discard(goal)
            segment_valid_cells.difference_update(local_reserved)

        if current_blocked_cells:
            local_blocked = set(current_blocked_cells)
            if blocked_exemption_radius > 0:
                local_blocked.difference_update(
                    dilate_cells({start, goal}, blocked_exemption_radius)
                )
            local_blocked.discard(start)
            local_blocked.discard(goal)
            segment_valid_cells.difference_update(local_blocked)

        # Enforce wire-to-node edge-to-edge clearance for nodes that
        # are NOT the current segment's endpoints.  reserved_cells
        # blocks only the physical node volume; this extends that by
        # blocked_radius so the conductive wire maintains at least
        # PATH_SEPARATION_MM gap from every non-connected node.
        if per_node_keepout:
            endpoint_safe = dilate_cells(
                {start, goal}, blocked_exemption_radius
            )
            for node in node_sequence:
                if node == start or node == goal:
                    continue
                if node in per_node_keepout:
                    segment_valid_cells.difference_update(
                        per_node_keepout[node] - endpoint_safe
                    )
            # Ensure start/goal always remain routable.
            segment_valid_cells.add(start)
            segment_valid_cells.add(goal)

        local_penalties = set(current_penalty_cells)
        local_penalties.discard(start)
        local_penalties.discard(goal)

        # Soft approach-direction penalty: discourage departing from
        # *start* in the same direction a previous segment arrived.
        prev_dir = approach_directions.get(start)
        if prev_dir is not None:
            # Penalise cells in the direction the previous segment came from
            # (i.e. the reversed arrival direction).  This nudges A* to
            # leave the node in a different direction.
            for depth in range(1, max(2, blocked_radius + 1)):
                penalised = (
                    start[0] + prev_dir[0] * depth,
                    start[1] + prev_dir[1] * depth,
                    start[2] + prev_dir[2] * depth,
                )
                local_penalties.add(penalised)

        segment_target_length = None
        if segment_target_lengths is not None:
            segment_target_length = segment_target_lengths[segment_index]

        try:
            candidate_paths = _segment_candidate_paths(
                valid_cells=segment_valid_cells,
                start=start,
                goal=goal,
                penalty_cells=local_penalties,
                penalty_weight=penalty_weight,
                allow_diagonals=allow_diagonals,
                target_length=segment_target_length,
                self_avoid_radius=max(0, blocked_radius),
                vertical_move_penalty=vertical_move_penalty,
                roominess_map=roominess_map,
                bottleneck_threshold=bottleneck_threshold,
                boundary_penalty_cells=boundary_penalty_cells,
                boundary_penalty_weight=0.2,
                beam_width=seg_beam_width,
                beam_max_waypoints=seg_beam_max_waypoints,
                deadline=deadline,
                min_self_spacing=min_self_spacing,
            )
        except RoutingError:
            candidate_paths = []

        # Pre-compute blocked zone from non-adjacent prior segments
        # (segments that share no start/goal node with the current one).
        # The blocked_exemption_radius around start/goal allows A* to
        # route through previously-blocked territory near those nodes,
        # but this can cause a path to overlap with a non-adjacent
        # earlier segment.  This set catches that case cheaply.
        non_adjacent_blocked: Optional[Set[GridIndex]] = None
        if blocked_radius > 0 and len(current_segments) >= 2:
            non_adj_cells: Set[GridIndex] = set()
            for prev_idx in range(len(current_segments) - 1):
                non_adj_cells.update(current_segments[prev_idx])
            non_adj_cells.discard(start)
            non_adj_cells.discard(goal)
            if non_adj_cells:
                non_adjacent_blocked = dilate_cells(non_adj_cells, blocked_radius)
                # Exempt cells near start/goal so the path can still
                # reach its endpoints.  Using only 1 cell was too tight
                # and prevented valid routes in dense layouts.
                node_exempt = dilate_cells({start, goal}, max(2, blocked_radius))
                non_adjacent_blocked.difference_update(node_exempt)

        last_error: Optional[RoutingError] = None
        for routed_segment in candidate_paths:
            # Cross-segment overlap check: verify the new segment does
            # not run parallel to the previous segment near the shared
            # node.  The blocked_exemption_radius is deliberately NOT
            # used here — only a tiny 1-cell zone around the shared node
            # is exempt so that near-node overlaps are caught.
            if blocked_radius > 0 and current_segments:
                prev_seg = current_segments[-1]
                shared = start
                if _cross_segment_near_approach(
                    prev_seg, routed_segment, blocked_radius, shared,
                    node_adjacency=max(2, blocked_exemption_radius),
                ):
                    continue

            # Non-adjacent segment overlap check: reject candidates that
            # wander through the exemption zone into territory blocked by
            # segments that do not directly connect to start or goal.
            if non_adjacent_blocked is not None:
                if any(cell in non_adjacent_blocked for cell in routed_segment):
                    continue

            next_penalty_cells = set(current_penalty_cells)
            next_penalty_cells.update(dilate_cells(routed_segment, penalty_radius))
            next_penalty_cells.discard(start)
            next_penalty_cells.discard(goal)

            next_blocked_cells = set(current_blocked_cells)
            if blocked_radius > 0:
                next_blocked_cells.update(dilate_cells(routed_segment, blocked_radius))

            # Record how this segment arrives at *goal* so the next
            # segment can prefer a different departure direction.
            next_approach = dict(approach_directions)
            arrival_dir = _approach_direction(routed_segment, from_end=True)
            if arrival_dir is not None:
                next_approach[goal] = arrival_dir

            try:
                return _search(
                    segment_index + 1,
                    next_penalty_cells,
                    next_blocked_cells,
                    current_segments + [compress_index_path(routed_segment)],
                    next_approach,
                )
            except RoutingError as error:
                last_error = error

        if last_error is not None:
            raise last_error

        if node_labels is not None:
            raise RoutingError(
                "Pathway between {} and {} could not fit the current routing constraints.".format(
                    node_labels[segment_index],
                    node_labels[segment_index + 1],
                )
            )
        raise RoutingError(
            "No route found between {} and {}.".format(start, goal)
        )

    return _search(0, set(), set(), segments, {})

