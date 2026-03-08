from __future__ import annotations

import heapq
import math
from dataclasses import dataclass
from typing import Dict, Iterable, Iterator, List, Optional, Sequence, Set, Tuple

GridIndex = Tuple[int, int, int]
FloatPoint = Tuple[float, float, float]


class RoutingError(RuntimeError):
    """Raised when a valid route cannot be found."""


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


def _local_roominess(valid_cells: Set[GridIndex], center: GridIndex, radius: int = 2) -> int:
    roominess = 0
    for dx in range(-radius, radius + 1):
        for dy in range(-radius, radius + 1):
            for dz in range(-radius, radius + 1):
                candidate = (center[0] + dx, center[1] + dy, center[2] + dz)
                if candidate in valid_cells:
                    roominess += 1
    return roominess


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

    for current_index, current in enumerate(path):
        for previous_index in range(0, max(0, current_index - local_window)):
            previous = path[previous_index]
            if max(
                abs(current[0] - previous[0]),
                abs(current[1] - previous[1]),
                abs(current[2] - previous[2]),
            ) <= radius:
                return True
    return False


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

    blocked_cells = dilate_cells(occupied_cells, self_avoid_radius)
    if internal & blocked_cells:
        return False

    return not _has_nonlocal_close_approach(candidate, self_avoid_radius)


def _grow_path_toward_target(
    path: Sequence[GridIndex],
    valid_cells: Set[GridIndex],
    target_length: float,
    self_avoid_radius: int,
) -> List[GridIndex]:
    grown = list(path)
    if len(grown) < 2:
        return grown

    current_length = _path_length(grown)
    if current_length + 1e-9 >= target_length:
        return grown

    max_iterations = max(32, int(target_length - current_length) * 2)
    for _ in range(max_iterations):
        if current_length + 1e-9 >= target_length:
            break

        edge_indices = sorted(
            range(len(grown) - 1),
            key=lambda index: min(
                _local_roominess(valid_cells, grown[index]),
                _local_roominess(valid_cells, grown[index + 1]),
            ),
            reverse=True,
        )

        best_replacement: Optional[List[GridIndex]] = None
        best_score: Optional[Tuple[float, bool, int, int, int, float]] = None
        for edge_index in edge_indices:
            start = grown[edge_index]
            goal = grown[edge_index + 1]
            if _segment_step_offset(start, goal) is None:
                continue

            local_exemption = dilate_cells({start, goal}, max(1, self_avoid_radius + 1))
            local_occupied = {cell for cell in grown if cell not in local_exemption}

            for candidate in _candidate_edge_detours(
                start,
                goal,
                max_primary_depth=6,
                max_secondary_depth=4,
            ):
                if not _is_valid_detour_candidate(
                    candidate,
                    valid_cells,
                    local_occupied,
                    self_avoid_radius,
                ):
                    continue

                expanded_path = grown[:edge_index] + candidate + grown[edge_index + 2 :]
                if _has_nonlocal_close_approach(expanded_path, self_avoid_radius):
                    continue

                expanded_length = _path_length(expanded_path)
                score = (
                    abs(expanded_length - target_length),
                    expanded_length < target_length,
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
) -> List[GridIndex]:
    ranked: List[Tuple[Tuple[float, float, float, float], GridIndex]] = []
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
        roominess = _local_roominess(valid_cells, candidate)
        midplane_offset = abs((candidate[2] * 2) - (start[2] + goal[2]))
        score = (
            abs(total_distance - target_length),
            midplane_offset,
            -roominess,
            -line_distance,
            -min(start_distance, goal_distance),
            -total_distance,
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
        best_score: Optional[Tuple[float, float, float]] = None
        for pool_index, candidate in enumerate(pool):
            if candidate in selected_set:
                continue
            diversity = min(_distance(candidate, existing) for existing in selected)
            line_distance = _point_to_segment_distance(candidate, start, goal)
            roominess = _local_roominess(valid_cells, candidate)
            score = (roominess, diversity, line_distance, -float(pool_index))
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
        )

        if combined:
            combined.extend(segment[1:])
        else:
            combined.extend(segment)
        consumed_cells.update(segment[1:-1])

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
) -> List[List[GridIndex]]:
    if target_length is None:
        return sorted((list(path) for path in paths), key=_path_length)

    return sorted(
        (list(path) for path in paths),
        key=lambda path: (
            abs(_path_length(path) - target_length),
            _path_length(path) < target_length,
            _path_vertical_span(path),
            _path_layer_count(path),
            _path_xy_footprint(path),
            -_path_length(path),
        ),
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
    max_candidates: int = 14,
    vertical_move_penalty: float = 0.0,
) -> List[List[GridIndex]]:
    direct_path = astar_path(
        valid_cells=valid_cells,
        start=start,
        goal=goal,
        penalty_cells=penalty_cells,
        penalty_weight=penalty_weight,
        allow_diagonals=allow_diagonals,
        vertical_move_penalty=vertical_move_penalty,
    )

    candidates: List[List[GridIndex]] = [direct_path]
    if target_length is None:
        return candidates

    direct_length = _path_length(direct_path)
    candidate_waypoints = _select_detour_waypoints(
        valid_cells=valid_cells,
        start=start,
        goal=goal,
        direct_length=direct_length,
        target_length=target_length,
        limit=18,
        allow_diagonals=allow_diagonals,
    )

    beam: List[Tuple[GridIndex, ...]] = [tuple()]
    beam_width = 14
    max_waypoints = 4
    for _ in range(max_waypoints):
        expanded: List[Tuple[Tuple[float, bool, float, float], Tuple[GridIndex, ...]]] = []
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
                            abs(estimated_length - target_length),
                            estimated_length < target_length,
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
            try:
                candidates.append(
                    _route_through_waypoints(
                        valid_cells=valid_cells,
                        waypoint_sequence=(start,) + waypoint_sequence + (goal,),
                        penalty_cells=penalty_cells,
                        penalty_weight=penalty_weight,
                        allow_diagonals=allow_diagonals,
                        self_avoid_radius=self_avoid_radius,
                        vertical_move_penalty=vertical_move_penalty,
                    )
                )
            except RoutingError:
                continue

    unique_candidates = _dedupe_paths(candidates)
    if target_length is not None and not allow_diagonals:
        grown_candidates = [
            _grow_path_toward_target(
                path,
                valid_cells,
                target_length,
                max(0, self_avoid_radius),
            )
            for path in unique_candidates
        ]
        unique_candidates = _dedupe_paths(unique_candidates + grown_candidates)
    return _sort_candidate_paths(unique_candidates, target_length)[:max_candidates]


def _neighbor_offsets(allow_diagonals: bool) -> Iterator[Tuple[GridIndex, float]]:
    if allow_diagonals:
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for dz in (-1, 0, 1):
                    if dx == 0 and dy == 0 and dz == 0:
                        continue
                    yield (dx, dy, dz), math.sqrt(dx * dx + dy * dy + dz * dz)
        return

    axial_offsets = (
        ((1, 0, 0), 1.0),
        ((-1, 0, 0), 1.0),
        ((0, 1, 0), 1.0),
        ((0, -1, 0), 1.0),
        ((0, 0, 1), 1.0),
        ((0, 0, -1), 1.0),
    )
    for offset, cost in axial_offsets:
        yield offset, cost


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
) -> List[GridIndex]:
    if start not in valid_cells:
        raise RoutingError(f"Start cell {start} is not inside the valid routing grid.")
    if goal not in valid_cells:
        raise RoutingError(f"Goal cell {goal} is not inside the valid routing grid.")
    if start == goal:
        return [start]

    penalties = penalty_cells or set()
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

    dilated: Set[GridIndex] = set()
    for cx, cy, cz in cells:
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                for dz in range(-radius, radius + 1):
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
) -> List[List[GridIndex]]:
    if len(node_sequence) < 2:
        raise RoutingError("At least two nodes are required to build a route.")

    if segment_target_lengths is not None and len(segment_target_lengths) != (len(node_sequence) - 1):
        raise RoutingError("Segment target lengths must match the routed segment count.")

    if node_labels is not None and len(node_labels) != len(node_sequence):
        raise RoutingError("Node labels must match the routed node sequence length.")

    segments: List[List[GridIndex]] = []
    reserved = reserved_cells or set()

    def _search(
        segment_index: int,
        current_penalty_cells: Set[GridIndex],
        current_blocked_cells: Set[GridIndex],
        current_segments: List[List[GridIndex]],
    ) -> List[List[GridIndex]]:
        if segment_index >= len(node_sequence) - 1:
            return list(current_segments)

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

        local_penalties = set(current_penalty_cells)
        local_penalties.discard(start)
        local_penalties.discard(goal)

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
            )
        except RoutingError:
            candidate_paths = []

        last_error: Optional[RoutingError] = None
        for routed_segment in candidate_paths:
            next_penalty_cells = set(current_penalty_cells)
            next_penalty_cells.update(dilate_cells(routed_segment, penalty_radius))
            next_penalty_cells.discard(start)
            next_penalty_cells.discard(goal)

            next_blocked_cells = set(current_blocked_cells)
            if blocked_radius > 0:
                next_blocked_cells.update(dilate_cells(routed_segment, blocked_radius))

            try:
                return _search(
                    segment_index + 1,
                    next_penalty_cells,
                    next_blocked_cells,
                    current_segments + [compress_index_path(routed_segment)],
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

    return _search(0, set(), set(), segments)
