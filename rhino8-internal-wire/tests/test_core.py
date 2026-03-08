from pathlib import Path
import sys
import unittest

LIBRARIES = Path(__file__).resolve().parents[1] / "Libraries"
if str(LIBRARIES) not in sys.path:
    sys.path.insert(0, str(LIBRARIES))

from wire_router.core import route_node_sequence


class CoreRouterTests(unittest.TestCase):
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


if __name__ == "__main__":
    unittest.main()
