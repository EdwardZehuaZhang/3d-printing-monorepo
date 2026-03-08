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


def route_node_sequence(
    valid_cells: Set[GridIndex],
    node_sequence: Sequence[GridIndex],
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

    if node_labels is not None and len(node_labels) != len(node_sequence):
        raise RoutingError("Node labels must match the routed node sequence length.")

    segments: List[List[GridIndex]] = []
    penalty_cells: Set[GridIndex] = set()
    blocked_cells: Set[GridIndex] = set()
    reserved = reserved_cells or set()

    for segment_index, (start, goal) in enumerate(zip(node_sequence[:-1], node_sequence[1:])):
        segment_valid_cells = set(valid_cells)
        if reserved:
            local_reserved = set(reserved)
            local_reserved.difference_update(
                dilate_cells({start, goal}, max(0, reserved_exemption_radius))
            )
            local_reserved.discard(start)
            local_reserved.discard(goal)
            segment_valid_cells.difference_update(local_reserved)

        if blocked_cells:
            local_blocked = set(blocked_cells)
            if blocked_exemption_radius > 0:
                local_blocked.difference_update(
                    dilate_cells({start, goal}, blocked_exemption_radius)
                )
            segment_valid_cells.difference_update(local_blocked)

        local_penalties = set(penalty_cells)
        local_penalties.discard(start)
        local_penalties.discard(goal)

        try:
            segment = astar_path(
                valid_cells=segment_valid_cells,
                start=start,
                goal=goal,
                penalty_cells=local_penalties,
                penalty_weight=penalty_weight,
                allow_diagonals=allow_diagonals,
            )
        except RoutingError:
            if node_labels is not None:
                raise RoutingError(
                    "Pathway between {} and {} is not big enough for the standardized conductive path and casing.".format(
                        node_labels[segment_index],
                        node_labels[segment_index + 1],
                    )
                )
            raise

        segment = compress_index_path(segment)
        segments.append(segment)

        penalty_cells.update(dilate_cells(segment, penalty_radius))
        penalty_cells.discard(start)
        penalty_cells.discard(goal)
        blocked_cells.update(dilate_cells(segment, blocked_radius))

    return segments
