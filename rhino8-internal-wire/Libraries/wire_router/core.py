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


def _path_length(path: Sequence[GridIndex]) -> float:
    if len(path) < 2:
        return 0.0
    return sum(_distance(start, end) for start, end in zip(path[:-1], path[1:]))


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
) -> List[GridIndex]:
    ranked: List[Tuple[Tuple[float, float, float, float], GridIndex]] = []
    for candidate in valid_cells:
        if candidate == start or candidate == goal:
            continue

        start_distance = _distance(start, candidate)
        goal_distance = _distance(candidate, goal)
        if min(start_distance, goal_distance) < 2.0:
            continue

        total_distance = start_distance + goal_distance
        if total_distance <= direct_length + 2.0:
            continue

        line_distance = _point_to_segment_distance(candidate, start, goal)
        score = (
            abs(total_distance - target_length),
            -line_distance,
            -min(start_distance, goal_distance),
            -total_distance,
        )
        ranked.append((score, candidate))

    ranked.sort(key=lambda item: item[0])
    return [candidate for _, candidate in ranked[:limit]]


def _route_through_waypoints(
    valid_cells: Set[GridIndex],
    waypoint_sequence: Sequence[GridIndex],
    penalty_cells: Set[GridIndex],
    penalty_weight: float,
    allow_diagonals: bool,
) -> List[GridIndex]:
    combined: List[GridIndex] = []
    consumed_cells: Set[GridIndex] = set()

    for start, goal in zip(waypoint_sequence[:-1], waypoint_sequence[1:]):
        segment_valid_cells = set(valid_cells)
        if consumed_cells:
            segment_valid_cells.difference_update(consumed_cells)
            segment_valid_cells.add(start)
            segment_valid_cells.add(goal)

        segment_penalties = set(penalty_cells)
        segment_penalties.update(consumed_cells)
        segment_penalties.discard(start)
        segment_penalties.discard(goal)

        segment = astar_path(
            valid_cells=segment_valid_cells,
            start=start,
            goal=goal,
            penalty_cells=segment_penalties,
            penalty_weight=penalty_weight,
            allow_diagonals=allow_diagonals,
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
) -> List[GridIndex]:
    direct_path = astar_path(
        valid_cells=valid_cells,
        start=start,
        goal=goal,
        penalty_cells=penalty_cells,
        penalty_weight=penalty_weight,
        allow_diagonals=allow_diagonals,
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
    )

    for waypoint in candidate_waypoints:
        try:
            candidate_path = _route_through_waypoints(
                valid_cells=valid_cells,
                waypoint_sequence=(start, waypoint, goal),
                penalty_cells=penalty_cells,
                penalty_weight=penalty_weight,
                allow_diagonals=allow_diagonals,
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
            -_path_length(path),
        ),
    )


def _segment_candidate_paths(
    valid_cells: Set[GridIndex],
    start: GridIndex,
    goal: GridIndex,
    penalty_cells: Set[GridIndex],
    penalty_weight: float,
    allow_diagonals: bool,
    target_length: Optional[float],
    max_candidates: int = 14,
) -> List[List[GridIndex]]:
    direct_path = astar_path(
        valid_cells=valid_cells,
        start=start,
        goal=goal,
        penalty_cells=penalty_cells,
        penalty_weight=penalty_weight,
        allow_diagonals=allow_diagonals,
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
    )

    for waypoint in candidate_waypoints:
        try:
            candidates.append(
                _route_through_waypoints(
                    valid_cells=valid_cells,
                    waypoint_sequence=(start, waypoint, goal),
                    penalty_cells=penalty_cells,
                    penalty_weight=penalty_weight,
                    allow_diagonals=allow_diagonals,
                )
            )
        except RoutingError:
            continue

    pair_candidates = candidate_waypoints[: min(6, len(candidate_waypoints))]
    for first_waypoint in pair_candidates:
        for second_waypoint in pair_candidates:
            if first_waypoint == second_waypoint:
                continue
            try:
                candidates.append(
                    _route_through_waypoints(
                        valid_cells=valid_cells,
                        waypoint_sequence=(start, first_waypoint, second_waypoint, goal),
                        penalty_cells=penalty_cells,
                        penalty_weight=penalty_weight,
                        allow_diagonals=allow_diagonals,
                    )
                )
            except RoutingError:
                continue

    unique_candidates = _dedupe_paths(candidates)
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
