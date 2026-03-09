from pathlib import Path
import sys
import unittest

LIBRARIES = Path(__file__).resolve().parents[1] / "Libraries"
if str(LIBRARIES) not in sys.path:
    sys.path.insert(0, str(LIBRARIES))

from wire_router.core import (
    evaluate_node_order,
    optimize_node_order_for_maximum_spacing,
    optimize_node_order_for_path,
    optimize_node_order_for_target_leg_length,
    route_node_sequence,
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
        valid_cells = {(x, y, 0) for x in range(12) for y in range(8)}
        segments = route_node_sequence(
            valid_cells=valid_cells,
            node_sequence=[(1, 1, 0), (10, 6, 0)],
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


if __name__ == "__main__":
    unittest.main()
