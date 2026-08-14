#!/usr/bin/env python3
import contextlib
import io
import json
import os
import pathlib
import sys
import tempfile
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import build_topology as topology


class TopologyBuilderTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = self.temporary.name

    def tearDown(self):
        self.temporary.cleanup()

    def write_map(self, width, height, resolution, obstacles=()):
        pixels = [[254 for _x in range(width)] for _y in range(height)]
        for mx, my in obstacles:
            raster_y = height - 1 - my
            pixels[raster_y][mx] = 0
        pgm = os.path.join(self.root, "map.pgm")
        with open(pgm, "w", encoding="ascii") as handle:
            handle.write(f"P2\n{width} {height}\n255\n")
            for row in pixels:
                handle.write(" ".join(str(value) for value in row) + "\n")
        yaml = os.path.join(self.root, "map.yaml")
        with open(yaml, "w", encoding="utf-8") as handle:
            handle.write(
                "image: map.pgm\n"
                f"resolution: {resolution}\n"
                "origin: [0.0, 0.0, 0.0]\n"
                "negate: 0\n"
                "occupied_thresh: 0.65\n"
                "free_thresh: 0.196\n"
            )
        return yaml

    def write_positions(self, points):
        path = os.path.join(self.root, "robot_positions.txt")
        with open(path, "w", encoding="utf-8") as handle:
            for x, y, label in points:
                suffix = f" {label}" if label else ""
                handle.write(f"{x} {y} 0 0 0 0{suffix}\n")
        return path

    def args(self, positions, map_yaml, *extra):
        return topology.parse_args([
            "--positions", positions,
            "--map-yaml", map_yaml,
            "--audit",
            "--minimum-clearance", "0.15",
            "--max-pgm-edge-distance", "3.0",
            "--min-node-degree", "1",
            "--max-node-degree", "4",
            *extra,
        ])

    def test_pgm_rows_are_flipped_into_ros_map_coordinates(self):
        yaml = self.write_map(5, 5, 1.0, obstacles={(1, 0)})
        model = topology.load_map_model(yaml)
        blocked_pose = {"x": 1.5, "y": 0.5}
        free_pose = {"x": 2.5, "y": 2.5}
        self.assertEqual(topology.point_clearance(blocked_pose, model), 0.0)
        self.assertGreater(topology.point_clearance(free_pose, model), 0.0)

    def test_file_order_does_not_create_edges(self):
        yaml = self.write_map(60, 30, 0.1, obstacles={(0, 0)})
        # The middle coordinate is deliberately the last line.  Sequential
        # logic would try P0-P1 (2 m); unordered geometry must choose P0-P2 and
        # P1-P2 (1 m each) under the 1.1 m edge limit.
        positions = self.write_positions([
            (1.0, 1.0, "W1"),
            (3.0, 1.0, "W2"),
            (2.0, 1.0, ""),
        ])
        args = self.args(
            positions, yaml, "--max-pgm-edge-distance", "1.1"
        )
        _poses, edges, report = topology.build_topology(positions, yaml, args)
        endpoints = {(edge["from"], edge["to"]) for edge in edges}
        self.assertEqual(endpoints, {(0, 2), (1, 2)})
        self.assertEqual(report["order_assumption"], "unordered_nodes")

    def test_obstacle_rejects_the_only_candidate_edge(self):
        obstacles = {(20, y) for y in range(30)}
        yaml = self.write_map(40, 30, 0.1, obstacles=obstacles)
        positions = self.write_positions([
            (1.0, 1.0, "W1"),
            (3.0, 1.0, "W2"),
        ])
        with self.assertRaisesRegex(RuntimeError, "No safe topology edge"):
            topology.build_topology(positions, yaml, self.args(positions, yaml))

    def test_natural_directional_neighbor_is_preserved(self):
        yaml = self.write_map(50, 40, 0.1, obstacles={(0, 0)})
        positions = self.write_positions([
            (1.0, 1.0, "W1"),
            (3.0, 1.0, "W2"),
            # This point lies inside the diameter circle of P0-P1, so the
            # Gabriel rule alone would remove P0-P1.  It is in another angular
            # sector, therefore the obvious horizontal corridor must remain.
            (1.6, 1.7, ""),
        ])
        args = self.args(positions, yaml)
        _poses, edges, _report = topology.build_topology(positions, yaml, args)
        by_endpoint = {(edge["from"], edge["to"]): edge for edge in edges}
        self.assertIn((0, 1), by_endpoint)
        self.assertIn(
            "directional_local", by_endpoint[(0, 1)]["selection_reasons"]
        )

    def test_audit_does_not_replace_topology(self):
        yaml = self.write_map(60, 40, 0.1, obstacles={(0, 0)})
        positions = self.write_positions([
            (1.0, 1.0, "W1"),
            (2.0, 1.0, ""),
            (1.5, 2.0, "W2"),
        ])
        output = os.path.join(self.root, "topology.yaml")
        report_path = os.path.join(self.root, "audit.json")
        with open(output, "w", encoding="utf-8") as handle:
            handle.write("sentinel\n")
        with contextlib.redirect_stdout(io.StringIO()):
            result = topology.main([
                "--positions", positions,
                "--map-yaml", yaml,
                "--output", output,
                "--report", report_path,
                "--audit",
                "--minimum-clearance", "0.15",
                "--min-node-degree", "1",
            ])
        self.assertEqual(result, 0)
        with open(output, "r", encoding="utf-8") as handle:
            self.assertEqual(handle.read(), "sentinel\n")
        with open(report_path, "r", encoding="utf-8") as handle:
            report = json.load(handle)
        self.assertEqual(report["mode"], "audit")
        self.assertEqual(report["component_count"], 1)

    def test_atomic_write_replaces_complete_file(self):
        path = os.path.join(self.root, "result.yaml")
        topology.atomic_write(path, "complete\n")
        with open(path, "r", encoding="utf-8") as handle:
            self.assertEqual(handle.read(), "complete\n")
        leftovers = [name for name in os.listdir(self.root) if name.startswith(".topology.")]
        self.assertEqual(leftovers, [])


if __name__ == "__main__":
    unittest.main()
