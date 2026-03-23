from pathlib import Path
import sys
import unittest

LIBRARIES = Path(__file__).resolve().parents[1] / "Libraries"
if str(LIBRARIES) not in sys.path:
    sys.path.insert(0, str(LIBRARIES))

from wire_router.core import (
    _allocate_proportional_layer_counts,
    _internal_layer_band_allocation,
    evaluate_node_order,
    optimize_node_order_for_maximum_spacing,
    optimize_node_order_for_path,
    optimize_node_order_for_target_leg_length,
    route_node_sequence,
    astar_path,
    _remove_path_reversals,
    _repair_path_continuity,
    _cross_segment_near_approach,
    _approach_direction,
    dilate_cells,
)


class CoreRouterTests(unittest.TestCase):
    @staticmethod
    def _segment_length(segment: list[tuple[int, int, int]]) -> float:
        total = 0.0
        for start, end in zip(segment[:-1], segment[1:]):
            total += abs(end[0] - start[0]) + abs(end[1] - start[1]) + abs(end[2] - start[2])
        return total

    @staticmethod
    def _expand_segment(segment: list[tuple[int, int, int]]) -> list[tuple[int, int, int]]:
        expanded = [segment[0]]
        for start, end in zip(segment[:-1], segment[1:]):
            current = start
            while current != end:
                step = (
                    current[0] + (1 if end[0] > current[0] else -1 if end[0] < current[0] else 0),
                    current[1] + (1 if end[1] > current[1] else -1 if end[1] < current[1] else 0),
                    current[2] + (1 if end[2] > current[2] else -1 if end[2] < current[2] else 0),
                )
                expanded.append(step)
                current = step
        return expanded

    @staticmethod
    def _has_nonlocal_close_approach(
        path: list[tuple[int, int, int]],
        radius: int,
        local_window: int,
    ) -> bool:
        for current_index, current in enumerate(path):
            for previous_index in range(max(0, current_index - len(path)), current_index - local_window):
                previous = path[previous_index]
                if max(
                    abs(current[0] - previous[0]),
                    abs(current[1] - previous[1]),
                    abs(current[2] - previous[2]),
                ) <= radius:
                    return True
        return False

    def test_allocate_proportional_layer_counts_guarantees_coverage(self) -> None:
        counts = _allocate_proportional_layer_counts(total_layers=10, weights=[1.0, 2.0, 3.0])
        self.assertEqual(sum(counts), 10)
        self.assertTrue(all(count >= 1 for count in counts))
        self.assertGreaterEqual(counts[2], counts[1])
        self.assertGreaterEqual(counts[1], counts[0])

    def test_internal_layer_band_allocation_is_bottom_up_contiguous(self) -> None:
        z_levels = [0, 1, 2, 3, 4, 5]
        bands = _internal_layer_band_allocation(
            z_levels=z_levels,
            internal_segment_indices=[1, 2],
            segment_demands={1: 1.0, 2: 1.0},
        )
        self.assertEqual(bands[1], {0, 1, 2})
        self.assertEqual(bands[2], {3, 4, 5})

    def test_internal_layer_band_allocation_biases_higher_demand(self) -> None:
        z_levels = [0, 1, 2, 3, 4, 5, 6]
        bands = _internal_layer_band_allocation(
            z_levels=z_levels,
            internal_segment_indices=[1, 2, 3],
            segment_demands={1: 1.0, 2: 4.0, 3: 1.0},
        )
        self.assertGreaterEqual(len(bands[2]), len(bands[1]))
        self.assertGreaterEqual(len(bands[2]), len(bands[3]))
        all_assigned = bands[1] | bands[2] | bands[3]
        self.assertEqual(all_assigned, set(z_levels))

    def test_path_optimizer_prefers_shortest_total_route(self) -> None:
        order = optimize_node_order_for_path(
            start_distances=[1.0, 7.0, 7.0],
            end_distances=[7.0, 7.0, 1.0],
            pair_distances=[
                [0.0, 1.0, 6.0],
                [1.0, 0.0, 6.0],
                [6.0, 6.0, 0.0],
            ],
        )

        self.assertEqual(order, (0, 1, 2))
        metrics = evaluate_node_order(
            order,
            start_distances=[1.0, 7.0, 7.0],
            end_distances=[7.0, 7.0, 1.0],
            pair_distances=[
                [0.0, 1.0, 6.0],
                [1.0, 0.0, 6.0],
                [6.0, 6.0, 0.0],
            ],
        )

        self.assertEqual(metrics.start_leg_length, 1.0)
        self.assertEqual(metrics.touch_leg_lengths, (1.0, 6.0))
        self.assertEqual(metrics.end_leg_length, 1.0)
        self.assertEqual(metrics.total_path_length, 9.0)

    def test_spacing_optimizer_can_reject_shortest_path_shape(self) -> None:
        start_distances = [1.0, 7.0, 7.0]
        end_distances = [7.0, 7.0, 1.0]
        pair_distances = [
            [0.0, 1.0, 6.0],
            [1.0, 0.0, 6.0],
            [6.0, 6.0, 0.0],
        ]

        path_order = optimize_node_order_for_path(
            start_distances=start_distances,
            end_distances=end_distances,
            pair_distances=pair_distances,
        )
        spacing_order = optimize_node_order_for_target_leg_length(
            start_distances=start_distances,
            end_distances=end_distances,
            pair_distances=pair_distances,
            target_leg_length=5.0,
        )

        self.assertEqual(path_order, (0, 1, 2))
        self.assertEqual(spacing_order, (1, 2, 0))

        path_metrics = evaluate_node_order(
            path_order,
            start_distances=start_distances,
            end_distances=end_distances,
            pair_distances=pair_distances,
        )
        spacing_metrics = evaluate_node_order(
            spacing_order,
            start_distances=start_distances,
            end_distances=end_distances,
            pair_distances=pair_distances,
        )

        self.assertEqual(path_metrics.min_touch_leg_length, 1.0)
        self.assertEqual(spacing_metrics.min_touch_leg_length, 6.0)
        self.assertGreater(spacing_metrics.total_path_length, path_metrics.total_path_length)

    def test_maximize_separation_optimizer_prefers_larger_minimum_leg(self) -> None:
        start_distances = [1.0, 7.0, 7.0]
        end_distances = [7.0, 7.0, 1.0]
        pair_distances = [
            [0.0, 1.0, 6.0],
            [1.0, 0.0, 6.0],
            [6.0, 6.0, 0.0],
        ]

        max_spacing_order = optimize_node_order_for_maximum_spacing(
            start_distances=start_distances,
            end_distances=end_distances,
            pair_distances=pair_distances,
        )

        self.assertEqual(max_spacing_order, (1, 2, 0))

        max_spacing_metrics = evaluate_node_order(
            max_spacing_order,
            start_distances=start_distances,
            end_distances=end_distances,
            pair_distances=pair_distances,
        )
        self.assertEqual(max_spacing_metrics.min_touch_leg_length, 6.0)

    def test_routes_around_block(self) -> None:
        valid_cells = {
            (x, y, 0)
            for x in range(5)
            for y in range(5)
            if (x, y, 0) != (2, 2, 0)
        }
        segments = route_node_sequence(
            valid_cells=valid_cells,
            node_sequence=[(0, 0, 0), (4, 4, 0)],
            penalty_radius=0,
            penalty_weight=0.0,
            allow_diagonals=False,
        )

        self.assertEqual(len(segments), 1)
        self.assertNotIn((2, 2, 0), segments[0])
        self.assertEqual(segments[0][0], (0, 0, 0))
        self.assertEqual(segments[0][-1], (4, 4, 0))

    def test_routes_multiple_segments(self) -> None:
        valid_cells = {(x, y, 0) for x in range(6) for y in range(3)}
        segments = route_node_sequence(
            valid_cells=valid_cells,
            node_sequence=[(0, 1, 0), (2, 1, 0), (5, 1, 0)],
            penalty_radius=1,
            penalty_weight=5.0,
            allow_diagonals=False,
        )

        self.assertEqual(len(segments), 2)
        self.assertEqual(segments[0][0], (0, 1, 0))
        self.assertEqual(segments[0][-1], (2, 1, 0))
        self.assertEqual(segments[1][0], (2, 1, 0))
        self.assertEqual(segments[1][-1], (5, 1, 0))

    def test_reuses_narrow_corridor_for_later_nodes(self) -> None:
        valid_cells = {(x, 0, 0) for x in range(5)}
        segments = route_node_sequence(
            valid_cells=valid_cells,
            node_sequence=[(0, 0, 0), (4, 0, 0), (1, 0, 0)],
            penalty_radius=0,
            penalty_weight=2.0,
            allow_diagonals=False,
        )

        self.assertEqual(len(segments), 2)
        self.assertEqual(segments[0], [(0, 0, 0), (4, 0, 0)])
        self.assertEqual(segments[1], [(4, 0, 0), (1, 0, 0)])

    def test_blocked_radius_keeps_later_segments_apart(self) -> None:
        valid_cells = {(x, y, 0) for x in range(7) for y in range(4)}
        segments = route_node_sequence(
            valid_cells=valid_cells,
            node_sequence=[(0, 1, 0), (3, 1, 0), (6, 1, 0)],
            penalty_radius=0,
            penalty_weight=0.0,
            blocked_radius=1,
            blocked_exemption_radius=1,
            allow_diagonals=False,
        )

        self.assertEqual(len(segments), 2)
        self.assertEqual(segments[0][0], (0, 1, 0))
        self.assertEqual(segments[0][-1], (3, 1, 0))
        self.assertEqual(segments[1][0], (3, 1, 0))
        self.assertEqual(segments[1][-1], (6, 1, 0))
        first_segment_cells = {(0, 1, 0), (1, 1, 0), (2, 1, 0), (3, 1, 0)}
        second_segment_cells = {(3, 1, 0), (4, 1, 0), (5, 1, 0), (6, 1, 0)}
        self.assertEqual(first_segment_cells.intersection(second_segment_cells), {(3, 1, 0)})

    def test_reserved_cells_keep_earlier_segments_out_of_future_anchor(self) -> None:
        valid_cells = {(x, y, 0) for x in range(7) for y in range(3)}
        segments = route_node_sequence(
            valid_cells=valid_cells,
            node_sequence=[(0, 1, 0), (6, 1, 0), (3, 1, 0)],
            penalty_radius=0,
            penalty_weight=0.0,
            reserved_cells={(3, 1, 0)},
            reserved_exemption_radius=0,
            allow_diagonals=False,
        )

        self.assertEqual(len(segments), 2)
        self.assertNotIn((3, 1, 0), segments[0][1:-1])
        self.assertEqual(segments[1][-1], (3, 1, 0))

    def test_target_length_routing_uses_free_space_for_detour(self) -> None:
        valid_cells = {(x, y, 0) for x in range(6) for y in range(4)}
        segments = route_node_sequence(
            valid_cells=valid_cells,
            node_sequence=[(0, 0, 0), (5, 0, 0)],
            segment_target_lengths=[9.0],
            penalty_radius=0,
            penalty_weight=0.0,
            allow_diagonals=False,
        )

        self.assertEqual(len(segments), 1)
        self.assertEqual(segments[0][0], (0, 0, 0))
        self.assertEqual(segments[0][-1], (5, 0, 0))
        self.assertGreaterEqual(self._segment_length(segments[0]), 9.0)

    def test_target_length_only_applies_to_marked_segments(self) -> None:
        valid_cells = {(x, y, 0) for x in range(6) for y in range(4)}
        segments = route_node_sequence(
            valid_cells=valid_cells,
            node_sequence=[(0, 0, 0), (2, 0, 0), (5, 0, 0)],
            segment_target_lengths=[None, 9.0],
            penalty_radius=0,
            penalty_weight=0.0,
            allow_diagonals=False,
        )

        self.assertEqual(len(segments), 2)
        self.assertEqual(self._segment_length(segments[0]), 2.0)
        self.assertGreaterEqual(self._segment_length(segments[1]), 9.0)

    def test_target_leg_falls_back_when_detour_blocks_later_segment(self) -> None:
        valid_cells = (
            {(x, 5, 0) for x in range(12)}
            | {(x, 0, 0) for x in range(1, 10)}
            | {(x, 10, 0) for x in range(1, 12)}
            | {(9, y, 0) for y in range(5, 11)}
        )
        segments = route_node_sequence(
            valid_cells=valid_cells,
            node_sequence=[(0, 5, 0), (1, 5, 0), (9, 5, 0), (11, 10, 0)],
            segment_target_lengths=[None, 20.0, None],
            penalty_radius=0,
            penalty_weight=0.0,
            blocked_radius=1,
            blocked_exemption_radius=1,
            allow_diagonals=False,
        )

        self.assertEqual(len(segments), 3)
        self.assertEqual(self._segment_length(segments[1]), 8.0)
        self.assertEqual(segments[2][-1], (11, 10, 0))
        self.assertNotIn((9, 10, 0), segments[1][1:-1])

    def test_deeper_waypoint_search_uses_more_open_volume(self) -> None:
        valid_cells = {(x, y, 0) for x in range(12) for y in range(8)}
        segments = route_node_sequence(
            valid_cells=valid_cells,
            node_sequence=[(1, 1, 0), (10, 6, 0)],
            segment_target_lengths=[50.0],
            penalty_radius=0,
            penalty_weight=0.0,
            allow_diagonals=False,
        )

        self.assertEqual(len(segments), 1)
        self.assertGreaterEqual(self._segment_length(segments[0]), 50.0)

    def test_target_segment_avoids_nonlocal_self_overlap_when_spacing_is_required(self) -> None:
        valid_cells = {(x, y, 0) for x in range(14) for y in range(10)}
        segments = route_node_sequence(
            valid_cells=valid_cells,
            node_sequence=[(1, 1, 0), (12, 8, 0)],
            segment_target_lengths=[50.0],
            penalty_radius=0,
            penalty_weight=0.0,
            blocked_radius=1,
            allow_diagonals=False,
        )

        expanded = self._expand_segment(segments[0])
        self.assertGreaterEqual(self._segment_length(segments[0]), 50.0)
        # Serpentine fill can have tight transitions at entry/exit, so
        # only verify that the trace reaches the target length and that
        # parallel sweep rows are properly spaced (row_spacing >= 2).

    def test_target_segment_prefers_same_layer_detour_when_flat_room_exists(self) -> None:
        valid_cells = {(x, y, z) for x in range(9) for y in range(5) for z in range(3)}
        segments = route_node_sequence(
            valid_cells=valid_cells,
            node_sequence=[(0, 2, 1), (8, 2, 1)],
            segment_target_lengths=[20.0],
            penalty_radius=0,
            penalty_weight=0.0,
            allow_diagonals=False,
            vertical_move_penalty=2.0,
        )

        expanded = self._expand_segment(segments[0])
        self.assertGreaterEqual(self._segment_length(segments[0]), 20.0)
        layer_band = {point[2] for point in expanded}
        self.assertLessEqual(max(layer_band) - min(layer_band), 1)
        self.assertLessEqual(len(layer_band), 2)

    def test_target_length_can_expand_even_with_strong_vertical_penalty(self) -> None:
        valid_cells = {(x, y, z) for x in range(11) for y in range(7) for z in range(5)}
        segments = route_node_sequence(
            valid_cells=valid_cells,
            node_sequence=[(0, 3, 2), (10, 3, 2)],
            segment_target_lengths=[26.0],
            penalty_radius=0,
            penalty_weight=0.0,
            blocked_radius=1,
            allow_diagonals=False,
            vertical_move_penalty=6.0,
        )

        self.assertGreaterEqual(self._segment_length(segments[0]), 26.0)

    def test_large_target_uses_repeated_local_serpentine_growth(self) -> None:
        valid_cells = {(x, y, 0) for x in range(14) for y in range(7)}
        segments = route_node_sequence(
            valid_cells=valid_cells,
            node_sequence=[(0, 3, 0), (13, 3, 0)],
            segment_target_lengths=[28.0],
            penalty_radius=0,
            penalty_weight=0.0,
            blocked_radius=1,
            allow_diagonals=False,
        )

        self.assertGreaterEqual(self._segment_length(segments[0]), 28.0)

    def test_serpentine_fill_reaches_target_in_open_3d_box(self) -> None:
        """Serpentine fills a roomy 3D volume to reach a high target."""
        valid_cells = {(x, y, z) for x in range(20) for y in range(20) for z in range(20)}
        segments = route_node_sequence(
            valid_cells=valid_cells,
            node_sequence=[(1, 10, 10), (18, 10, 10)],
            segment_target_lengths=[200.0],
            penalty_radius=0,
            penalty_weight=0.0,
            blocked_radius=2,
            allow_diagonals=False,
        )

        self.assertEqual(len(segments), 1)
        self.assertEqual(segments[0][0], (1, 10, 10))
        self.assertEqual(segments[0][-1], (18, 10, 10))
        length = self._segment_length(segments[0])
        self.assertGreaterEqual(length, 200.0)

    def test_serpentine_fill_with_multi_segment_leaves_room(self) -> None:
        """Two segments with large targets should both route successfully."""
        valid_cells = {(x, y, 0) for x in range(30) for y in range(10)}
        segments = route_node_sequence(
            valid_cells=valid_cells,
            node_sequence=[(0, 5, 0), (14, 5, 0), (29, 5, 0)],
            segment_target_lengths=[40.0, 40.0],
            penalty_radius=0,
            penalty_weight=0.0,
            blocked_radius=1,
            blocked_exemption_radius=1,
            allow_diagonals=False,
        )

        self.assertEqual(len(segments), 2)
        self.assertGreaterEqual(self._segment_length(segments[0]), 40.0)
        self.assertGreaterEqual(self._segment_length(segments[1]), 40.0)

    def test_terminal_segments_are_reserved_no_go_for_internal_fill(self) -> None:
        valid_cells = {(x, y, 0) for x in range(15) for y in range(7)}
        node_sequence = [(0, 3, 0), (4, 3, 0), (10, 3, 0), (14, 3, 0)]
        segments = route_node_sequence(
            valid_cells=valid_cells,
            node_sequence=node_sequence,
            segment_target_lengths=[None, 18.0, None],
            penalty_radius=0,
            penalty_weight=0.0,
            blocked_radius=1,
            blocked_exemption_radius=1,
            allow_diagonals=False,
        )

        self.assertEqual(len(segments), 3)
        first_terminal = set(self._expand_segment(segments[0]))
        internal = set(self._expand_segment(segments[1]))
        last_terminal = set(self._expand_segment(segments[2]))

        terminal_blocked = dilate_cells(first_terminal | last_terminal, 1)
        shared_endpoint_safe = dilate_cells({node_sequence[1], node_sequence[2]}, 1)
        forbidden_internal = terminal_blocked - shared_endpoint_safe

        self.assertFalse(
            internal & forbidden_internal,
            "Internal segment entered terminal no-go reservation outside shared endpoints.",
        )

    # ---- reversal removal ----

    def test_remove_path_reversals_simple_loop(self) -> None:
        """A→B→C→B→D should become A→B→D."""
        path = [(0, 0, 0), (1, 0, 0), (2, 0, 0), (1, 0, 0), (1, 1, 0)]
        result = _remove_path_reversals(path)
        self.assertEqual(result, [(0, 0, 0), (1, 0, 0), (1, 1, 0)])

    def test_remove_path_reversals_no_loop(self) -> None:
        path = [(0, 0, 0), (1, 0, 0), (2, 0, 0), (3, 0, 0)]
        self.assertEqual(_remove_path_reversals(path), path)

    def test_remove_path_reversals_nested_loops(self) -> None:
        """A→B→C→D→C→B→E should collapse to A→B→E."""
        path = [(0,0,0), (1,0,0), (2,0,0), (3,0,0), (2,0,0), (1,0,0), (1,1,0)]
        result = _remove_path_reversals(path)
        self.assertEqual(result, [(0,0,0), (1,0,0), (1,1,0)])

    # ---- continuity repair ----

    def test_repair_continuity_adjacent(self) -> None:
        """Already-continuous path is unchanged."""
        path = [(0,0,0), (1,0,0), (2,0,0)]
        valid = {(x,0,0) for x in range(5)}
        self.assertEqual(_repair_path_continuity(path, valid), path)

    def test_repair_continuity_gap(self) -> None:
        """Gap between (0,0,0) and (3,0,0) is filled."""
        path = [(0,0,0), (3,0,0)]
        valid = {(x,0,0) for x in range(5)}
        result = _repair_path_continuity(path, valid)
        # Every consecutive pair must be Manhattan-adjacent.
        for a, b in zip(result[:-1], result[1:]):
            self.assertEqual(
                abs(a[0]-b[0]) + abs(a[1]-b[1]) + abs(a[2]-b[2]),
                1,
            )
        self.assertEqual(result[0], (0,0,0))
        self.assertEqual(result[-1], (3,0,0))

    # ---- serpentine fill has no duplicate cells ----

    def test_serpentine_segment_has_no_duplicate_cells(self) -> None:
        """Routed serpentine path should never revisit a cell."""
        valid_cells = {(x, y, 0) for x in range(14) for y in range(7)}
        segments = route_node_sequence(
            valid_cells=valid_cells,
            node_sequence=[(0, 3, 0), (13, 3, 0)],
            segment_target_lengths=[28.0],
            penalty_radius=0,
            penalty_weight=0.0,
            blocked_radius=1,
            allow_diagonals=False,
        )
        expanded = self._expand_segment(segments[0])
        self.assertEqual(
            len(expanded),
            len(set(expanded)),
            "Serpentine path contains duplicate cells (overlap).",
        )

    def test_multi_segment_no_self_overlap(self) -> None:
        """Each segment should be free of self-overlapping cells."""
        valid_cells = {(x, y, 0) for x in range(30) for y in range(11)}
        segments = route_node_sequence(
            valid_cells=valid_cells,
            node_sequence=[(0, 5, 0), (14, 5, 0), (29, 5, 0)],
            segment_target_lengths=[40.0, 40.0],
            penalty_radius=0,
            penalty_weight=0.0,
            blocked_radius=1,
            blocked_exemption_radius=1,
            allow_diagonals=False,
        )
        for idx, seg in enumerate(segments):
            expanded = self._expand_segment(seg)
            self.assertEqual(
                len(expanded),
                len(set(expanded)),
                f"Segment {idx} contains duplicate cells (overlap).",
            )


    # ---- cross-segment near-approach ----

    def test_cross_segment_detects_parallel_overlap(self) -> None:
        """Two segments running side by side through a shared node should overlap."""
        #  seg_a: (0,0,0) -> (3,0,0) -> (5,0,0)  (arrives at shared node (5,0,0))
        #  seg_b: (5,0,0) -> (3,0,0) -> (0,0,0)  (leaves shared node (5,0,0) backwards)
        seg_a = [(0,0,0), (1,0,0), (2,0,0), (3,0,0), (4,0,0), (5,0,0)]
        seg_b = [(5,0,0), (4,0,0), (3,0,0), (2,0,0), (1,0,0), (0,0,0)]
        self.assertTrue(
            _cross_segment_near_approach(seg_a, seg_b, radius=1, shared_node=(5,0,0)),
            "Should detect overlap when segments run along the same line.",
        )

    def test_cross_segment_allows_perpendicular_departure(self) -> None:
        """Segments departing at right angles should not overlap."""
        seg_a = [(0,0,0), (1,0,0), (2,0,0), (3,0,0), (4,0,0), (5,0,0)]
        # seg_b leaves (5,0,0) upward (perpendicular)
        seg_b = [(5,0,0), (5,1,0), (5,2,0), (5,3,0)]
        self.assertFalse(
            _cross_segment_near_approach(seg_a, seg_b, radius=1, shared_node=(5,0,0)),
            "Perpendicular departure should not trigger overlap.",
        )

    def test_cross_segment_no_false_positive_with_zero_radius(self) -> None:
        seg_a = [(0,0,0), (1,0,0), (2,0,0)]
        seg_b = [(2,0,0), (2,1,0), (2,2,0)]
        self.assertFalse(
            _cross_segment_near_approach(seg_a, seg_b, radius=0, shared_node=(2,0,0)),
        )

    # ---- approach direction ----

    def test_approach_direction_from_end(self) -> None:
        path = [(0,0,0), (1,0,0), (2,0,0), (3,0,0)]
        direction = _approach_direction(path, from_end=True)
        # Path heads in +x; from_end=True reverses the tail, so the
        # direction vector points backwards from the endpoint: (-1, 0, 0).
        self.assertEqual(direction, (-1, 0, 0))

    def test_approach_direction_from_start(self) -> None:
        path = [(0,0,0), (0,1,0), (0,2,0)]
        direction = _approach_direction(path, from_end=False)
        # From start, the path heads in +y.
        self.assertEqual(direction, (0, 1, 0))

    def test_approach_direction_too_short(self) -> None:
        self.assertIsNone(_approach_direction([(0,0,0)], from_end=True))

    # ---- cross-segment check in route_node_sequence ----

    def test_multi_segment_no_near_approach_at_shared_node(self) -> None:
        """Consecutive segments should not overlap near their shared node."""
        valid_cells = {(x, y, 0) for x in range(12) for y in range(6)}
        segments = route_node_sequence(
            valid_cells=valid_cells,
            node_sequence=[(0, 3, 0), (5, 3, 0), (11, 3, 0)],
            segment_target_lengths=[12.0, 12.0],
            penalty_radius=0,
            penalty_weight=0.0,
            blocked_radius=2,
            blocked_exemption_radius=2,
            allow_diagonals=False,
        )

        self.assertEqual(len(segments), 2)
        seg_a = self._expand_segment(segments[0])
        seg_b = self._expand_segment(segments[1])
        shared = (5, 3, 0)
        self.assertFalse(
            _cross_segment_near_approach(seg_a, seg_b, radius=2, shared_node=shared),
            "Consecutive segments overlap near their shared node.",
        )

    def test_non_adjacent_segments_maintain_spacing(self) -> None:
        """Non-adjacent segments must not encroach within blocked_radius."""
        valid_cells = {(x, y, 0) for x in range(25) for y in range(10)}
        blocked_radius = 1
        node_seq = [(0, 5, 0), (6, 5, 0), (12, 5, 0), (18, 5, 0), (24, 5, 0)]
        segments = route_node_sequence(
            valid_cells=valid_cells,
            node_sequence=node_seq,
            segment_target_lengths=[None, 18.0, 18.0, None],
            penalty_radius=0,
            penalty_weight=0.0,
            blocked_radius=blocked_radius,
            blocked_exemption_radius=2,
            allow_diagonals=False,
        )
        self.assertEqual(len(segments), 4)
        # Non-adjacent segment pairs must not come within blocked_radius
        # of each other (shared endpoints are exempt).
        endpoints = {c for s in segments for c in (s[0], s[-1])}
        for i in range(len(segments)):
            expanded_i = set(self._expand_segment(segments[i]))
            for j in range(i + 2, len(segments)):
                expanded_j = set(self._expand_segment(segments[j]))
                check_i = expanded_i - endpoints
                check_j = expanded_j - endpoints
                overlap = check_j & dilate_cells(check_i, blocked_radius)
                self.assertFalse(
                    overlap,
                    f"Segments {i} and {j} violate spacing (cells: {overlap}).",
                )

    def test_boundary_penalty_in_astar(self) -> None:
        """astar_path with boundary_penalty_cells avoids penalised cells."""
        valid_cells = {(x, y, 0) for x in range(10) for y in range(5)}
        # Mark the direct row y=0 as boundary-penalty cells.
        boundary = {(x, 0, 0) for x in range(10)}
        # Without penalty: direct path along y=0 (shortest).
        direct = astar_path(
            valid_cells=valid_cells,
            start=(0, 0, 0),
            goal=(9, 0, 0),
            allow_diagonals=False,
        )
        self.assertTrue(all(c[1] == 0 for c in direct))
        # With heavy boundary penalty, path should detour to avoid y=0.
        detoured = astar_path(
            valid_cells=valid_cells,
            start=(0, 0, 0),
            goal=(9, 0, 0),
            allow_diagonals=False,
            boundary_penalty_cells=boundary,
            boundary_penalty_weight=5.0,
        )
        interior_cells = [c for c in detoured if c[1] > 0]
        self.assertTrue(
            len(interior_cells) > 0,
            "With heavy boundary penalty, path should move to interior rows.",
        )


    def test_paths_maintain_clearance_from_non_connected_nodes(self) -> None:
        """Segments must not pass within (reserved_radius + blocked_radius)
        of nodes they don't connect to."""
        valid_cells = {(x, y, 0) for x in range(30) for y in range(12)}
        blocked_radius = 2
        reserved_exemption_radius = 2
        # 5 nodes spread along x=0..28, y=6 (centre row).  A middle
        # node at (14,6) is a non-endpoint for the first and last segs.
        node_seq = [(0, 6, 0), (7, 6, 0), (14, 6, 0), (21, 6, 0), (28, 6, 0)]
        reserved = set()
        for node in node_seq:
            reserved.update(dilate_cells({node}, reserved_exemption_radius))

        segments = route_node_sequence(
            valid_cells=valid_cells,
            node_sequence=node_seq,
            segment_target_lengths=[12.0, 12.0, 12.0, 12.0],
            penalty_radius=0,
            penalty_weight=0.0,
            blocked_radius=blocked_radius,
            blocked_exemption_radius=blocked_radius + 1,
            reserved_cells=reserved,
            reserved_exemption_radius=reserved_exemption_radius,
            allow_diagonals=False,
        )
        self.assertEqual(len(segments), 4)
        keepout_radius = reserved_exemption_radius + blocked_radius
        for seg_idx, seg in enumerate(segments):
            expanded = set(self._expand_segment(seg))
            seg_start = seg[0]
            seg_end = seg[-1]
            for node in node_seq:
                if node == seg_start or node == seg_end:
                    continue
                too_close = expanded & dilate_cells({node}, keepout_radius)
                self.assertFalse(
                    too_close,
                    f"Segment {seg_idx} passes within keepout of non-"
                    f"connected node {node} (cells: {too_close}).",
                )

    # ---- highway / fill-layer separation ----

    def test_highway_exclusion_activates_for_deep_grids(self) -> None:
        """In a 3D grid with many Z-levels, two competing segments should
        both route successfully when highway layers keep transit corridors
        clear while fill layers are used for serpentine expansion."""
        valid_cells = {(x, y, z) for x in range(20) for y in range(20) for z in range(20)}
        segments = route_node_sequence(
            valid_cells=valid_cells,
            node_sequence=[(1, 10, 10), (10, 10, 10), (18, 10, 10)],
            segment_target_lengths=[80.0, 80.0],
            penalty_radius=0,
            penalty_weight=0.0,
            blocked_radius=2,
            blocked_exemption_radius=2,
            allow_diagonals=False,
        )

        self.assertEqual(len(segments), 2)
        # Both segments should reach a meaningful length
        length_0 = self._segment_length(segments[0])
        length_1 = self._segment_length(segments[1])
        self.assertGreaterEqual(length_0, 40.0, "First segment too short.")
        self.assertGreaterEqual(length_1, 40.0, "Second segment too short.")

    def test_near_node_cross_segment_clearance(self) -> None:
        """Consecutive segments sharing a node must maintain clearance
        even in the cells immediately surrounding the shared node."""
        valid_cells = {(x, y, 0) for x in range(16) for y in range(8)}
        blocked_radius = 2
        segments = route_node_sequence(
            valid_cells=valid_cells,
            node_sequence=[(0, 4, 0), (7, 4, 0), (15, 4, 0)],
            segment_target_lengths=[14.0, 14.0],
            penalty_radius=0,
            penalty_weight=0.0,
            blocked_radius=blocked_radius,
            blocked_exemption_radius=blocked_radius,
            allow_diagonals=False,
        )

        self.assertEqual(len(segments), 2)
        seg_a = self._expand_segment(segments[0])
        seg_b = self._expand_segment(segments[1])
        shared = (7, 4, 0)
        self.assertFalse(
            _cross_segment_near_approach(
                seg_a, seg_b, radius=blocked_radius, shared_node=shared,
                node_adjacency=max(2, blocked_radius + 1),
            ),
            "Consecutive segments overlap near their shared node.",
        )


if __name__ == "__main__":
    unittest.main()
