#!/usr/bin/env python3
"""Build and audit a safety-oriented sparse waypoint topology.

Waypoint file order is deliberately *not* interpreted as route order.  The
line number is only a stable node id; every edge must pass the same map and
clearance checks before it can enter the graph.
"""

import argparse
import json
import math
import os
import tempfile
from collections import Counter, deque

import cv2
import numpy as np


FNV64_OFFSET = 14695981039346656037
FNV64_PRIME = 1099511628211


def read_simple_yaml(path):
    values = {}
    with open(path, "r", encoding="utf-8") as handle:
        for raw_line in handle:
            line = raw_line.split("#", 1)[0].strip()
            if not line or ":" not in line:
                continue
            key, value = line.split(":", 1)
            key = key.strip()
            value = value.strip()
            if value.startswith("[") and value.endswith("]"):
                items = [item.strip() for item in value[1:-1].split(",") if item.strip()]
                values[key] = [float(item) for item in items]
            else:
                try:
                    values[key] = float(value)
                except ValueError:
                    values[key] = value.strip("'\"")
    return values


def read_pgm(path):
    with open(path, "rb") as handle:
        magic = handle.readline().strip()
        if magic not in (b"P5", b"P2"):
            raise ValueError("Only P5/P2 PGM maps are supported")

        tokens = []
        while len(tokens) < 3:
            line = handle.readline()
            if not line:
                raise ValueError("Unexpected EOF in PGM header")
            line = line.split(b"#", 1)[0].strip()
            if line:
                tokens.extend(line.split())

        width = int(tokens[0])
        height = int(tokens[1])
        max_value = int(tokens[2])
        if width <= 0 or height <= 0 or max_value <= 0 or max_value > 255:
            raise ValueError("Invalid PGM dimensions/max value")

        if magic == b"P5":
            data = list(handle.read(width * height))
        else:
            rest = handle.read().split()
            data = [int(value) for value in rest[: width * height]]

    if len(data) != width * height:
        raise ValueError("PGM data size does not match width/height")
    return width, height, max_value, data


def load_positions(path):
    poses = []
    labels = set()
    with open(path, "r", encoding="utf-8") as handle:
        for line_no, raw_line in enumerate(handle, 1):
            line = raw_line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) < 6:
                raise ValueError(f"Invalid pose at line {line_no}: {line}")
            values = [float(value) for value in parts[:6]]
            if not all(math.isfinite(value) for value in values):
                raise ValueError(f"Non-finite pose at line {line_no}: {line}")
            label = parts[6] if len(parts) > 6 and parts[6] != "-" else ""
            if label and label in labels:
                raise ValueError(f"Duplicate waypoint label {label!r} at line {line_no}")
            labels.add(label)
            poses.append({
                "id": len(poses),
                "x": values[0],
                "y": values[1],
                "z": values[2],
                "roll": values[3],
                "pitch": values[4],
                "yaw": values[5],
                "label": label,
            })
    return poses


def resolve_map_image(map_yaml_path, image):
    path = image if os.path.isabs(image) else os.path.join(os.path.dirname(map_yaml_path), image)
    if os.path.isabs(image) and not os.path.exists(path):
        fallback = os.path.join(os.path.dirname(map_yaml_path), os.path.basename(image))
        if os.path.exists(fallback):
            path = fallback
    return os.path.abspath(path)


def fingerprint_files(paths):
    value = FNV64_OFFSET
    for path in paths:
        with open(path, "rb") as handle:
            while True:
                chunk = handle.read(1024 * 1024)
                if not chunk:
                    break
                for byte in chunk:
                    value ^= byte
                    value = (value * FNV64_PRIME) & 0xFFFFFFFFFFFFFFFF
    return f"{value:016x}"


def world_to_map(x, y, origin, resolution):
    yaw = origin[2] if len(origin) > 2 else 0.0
    dx = x - origin[0]
    dy = y - origin[1]
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    local_x = cos_yaw * dx + sin_yaw * dy
    local_y = -sin_yaw * dx + cos_yaw * dy
    return (int(math.floor(local_x / resolution)),
            int(math.floor(local_y / resolution)))


def load_map_model(map_yaml_path, unknown_is_obstacle=True):
    values = read_simple_yaml(map_yaml_path)
    resolution = float(values["resolution"])
    if not math.isfinite(resolution) or resolution <= 0.0:
        raise ValueError("Map resolution must be positive")
    origin = list(values.get("origin", [0.0, 0.0, 0.0]))
    while len(origin) < 3:
        origin.append(0.0)
    occupied_thresh = float(values.get("occupied_thresh", 0.65))
    free_thresh = float(values.get("free_thresh", 0.196))
    negate = int(values.get("negate", 0))
    pgm_path = resolve_map_image(map_yaml_path, values.get("image", "map.pgm"))
    width, height, max_value, data = read_pgm(pgm_path)

    raster = np.asarray(data, dtype=np.float32).reshape(height, width)
    normalized = raster / float(max_value)
    occupancy = normalized if negate else (1.0 - normalized)
    occupied_raster = occupancy >= occupied_thresh
    unknown_raster = np.logical_and(occupancy > free_thresh, ~occupied_raster)
    blocked_raster = np.logical_or(
        occupied_raster,
        unknown_raster if unknown_is_obstacle else np.zeros_like(unknown_raster),
    )

    # PGM row zero is the top of the image; OccupancyGrid/map coordinates start
    # at the lower-left.  Store all grids in map-cell order from here onward.
    occupied = np.flipud(occupied_raster)
    unknown = np.flipud(unknown_raster)
    blocked = np.flipud(blocked_raster).copy()
    # The footprint must stay inside the known map extent.  Treating the map
    # boundary as an obstacle makes the clearance field enforce that rule.
    blocked[0, :] = True
    blocked[-1, :] = True
    blocked[:, 0] = True
    blocked[:, -1] = True
    free_mask = np.asarray(~blocked, dtype=np.uint8)
    clearance = cv2.distanceTransform(
        free_mask, cv2.DIST_L2, cv2.DIST_MASK_PRECISE
    ) * resolution

    return {
        "yaml_path": os.path.abspath(map_yaml_path),
        "pgm_path": pgm_path,
        "width": width,
        "height": height,
        "resolution": resolution,
        "origin": origin,
        "occupied": occupied,
        "unknown": unknown,
        "blocked": blocked,
        "clearance": clearance,
        "occupied_cells": int(np.count_nonzero(occupied)),
        "unknown_cells": int(np.count_nonzero(unknown)),
    }


def point_clearance(pose, map_model):
    mx, my = world_to_map(
        pose["x"], pose["y"], map_model["origin"], map_model["resolution"]
    )
    if mx < 0 or my < 0 or mx >= map_model["width"] or my >= map_model["height"]:
        return -1.0
    return float(map_model["clearance"][my, mx])


def evaluate_edge(a, b, map_model, required_clearance, sample_step):
    length = math.hypot(a["x"] - b["x"], a["y"] - b["y"])
    if length <= 1e-9:
        return {"valid": False, "reason": "duplicate", "length": length,
                "min_clearance": 0.0}

    # Sampling at no more than half a map cell prevents a one-cell obstacle
    # from being skipped even when the command-line sample step is larger.
    step_size = min(sample_step, map_model["resolution"] * 0.5)
    steps = max(1, int(math.ceil(length / max(1e-4, step_size))))
    minimum = math.inf
    for step in range(steps + 1):
        t = step / float(steps)
        x = a["x"] + (b["x"] - a["x"]) * t
        y = a["y"] + (b["y"] - a["y"]) * t
        mx, my = world_to_map(x, y, map_model["origin"], map_model["resolution"])
        if mx < 0 or my < 0 or mx >= map_model["width"] or my >= map_model["height"]:
            return {"valid": False, "reason": "outside_map", "length": length,
                    "min_clearance": -1.0}
        minimum = min(minimum, float(map_model["clearance"][my, mx]))
        if minimum + 1e-9 < required_clearance:
            return {"valid": False, "reason": "insufficient_clearance",
                    "length": length, "min_clearance": minimum}

    return {"valid": True, "reason": "accepted", "length": length,
            "min_clearance": minimum}


def edge_cost(length, min_clearance, clearance_weight):
    # Clearance is a soft preference after the hard footprint threshold.  A
    # slightly longer corridor can therefore beat a wall-hugging shortcut.
    return length * (1.0 + clearance_weight / max(0.01, min_clearance))


class UnionFind:
    def __init__(self, size):
        self.parent = list(range(size))
        self.rank = [0] * size

    def find(self, item):
        while self.parent[item] != item:
            self.parent[item] = self.parent[self.parent[item]]
            item = self.parent[item]
        return item

    def union(self, a, b):
        a = self.find(a)
        b = self.find(b)
        if a == b:
            return False
        if self.rank[a] < self.rank[b]:
            a, b = b, a
        self.parent[b] = a
        if self.rank[a] == self.rank[b]:
            self.rank[a] += 1
        return True


def graph_adjacency(node_count, edges, skip_node=None):
    adjacency = [set() for _ in range(node_count)]
    for edge in edges:
        a, b = edge["from"], edge["to"]
        if a == skip_node or b == skip_node:
            continue
        adjacency[a].add(b)
        adjacency[b].add(a)
    return adjacency


def graph_components(node_count, edges, skip_node=None):
    adjacency = graph_adjacency(node_count, edges, skip_node)
    unseen = {node for node in range(node_count) if node != skip_node}
    components = []
    while unseen:
        root = min(unseen)
        unseen.remove(root)
        queue = deque([root])
        component = {root}
        while queue:
            node = queue.popleft()
            for neighbor in adjacency[node]:
                if neighbor in unseen:
                    unseen.remove(neighbor)
                    component.add(neighbor)
                    queue.append(neighbor)
        components.append(component)
    return components


def graph_cut_analysis(node_count, edges):
    adjacency = graph_adjacency(node_count, edges)
    discovered = [-1] * node_count
    low = [0] * node_count
    parent = [-1] * node_count
    timer = [0]
    articulation = set()
    bridges = []

    def visit(node):
        discovered[node] = low[node] = timer[0]
        timer[0] += 1
        children = 0
        for neighbor in sorted(adjacency[node]):
            if discovered[neighbor] < 0:
                parent[neighbor] = node
                children += 1
                visit(neighbor)
                low[node] = min(low[node], low[neighbor])
                if parent[node] < 0 and children > 1:
                    articulation.add(node)
                if parent[node] >= 0 and low[neighbor] >= discovered[node]:
                    articulation.add(node)
                if low[neighbor] > discovered[node]:
                    bridges.append((min(node, neighbor), max(node, neighbor)))
            elif neighbor != parent[node]:
                low[node] = min(low[node], discovered[neighbor])

    for node in range(node_count):
        if discovered[node] < 0:
            visit(node)
    return sorted(articulation), sorted(set(bridges))


def select_sparse_edges(node_count, candidates, min_degree, max_degree):
    ordered = sorted(
        candidates,
        key=lambda edge: (edge["cost"], -edge["min_clearance"],
                          edge["length"], edge["from"], edge["to"]),
    )
    selected = []
    selected_keys = set()
    degree = [0] * node_count

    def add(edge, enforce_max=True):
        key = (edge["from"], edge["to"])
        if key in selected_keys:
            return False
        if enforce_max and (degree[key[0]] >= max_degree or degree[key[1]] >= max_degree):
            return False
        selected.append(edge)
        selected_keys.add(key)
        degree[key[0]] += 1
        degree[key[1]] += 1
        return True

    # A minimum-cost spanning tree guarantees connectivity without using file
    # order.  Degree limits are applied only to redundancy edges; connectivity
    # is more important than the cosmetic maximum degree.
    union_find = UnionFind(node_count)
    for edge in ordered:
        if union_find.union(edge["from"], edge["to"]):
            add(edge, enforce_max=False)
    if node_count > 0 and len(graph_components(node_count, selected)) != 1:
        raise RuntimeError("Safe candidate graph is disconnected")

    # Give each node local alternatives where geometry permits.
    for node in range(node_count):
        for edge in ordered:
            if degree[node] >= min_degree:
                break
            if node in (edge["from"], edge["to"]):
                add(edge)

    # Greedily bypass articulation points.  This specifically avoids a single
    # topo node/area becoming the only route between two otherwise safe zones.
    for _ in range(max(1, node_count * 2)):
        articulation, _bridges = graph_cut_analysis(node_count, selected)
        added = False
        for cut_node in articulation:
            components = graph_components(node_count, selected, skip_node=cut_node)
            component_of = {}
            for index, component in enumerate(components):
                for node in component:
                    component_of[node] = index
            alternatives = [
                edge for edge in ordered
                if edge["from"] != cut_node and edge["to"] != cut_node
                and (edge["from"], edge["to"]) not in selected_keys
                and component_of.get(edge["from"]) != component_of.get(edge["to"])
            ]
            for edge in alternatives:
                if add(edge):
                    added = True
                    break
        if not added:
            break
    return sorted(selected, key=lambda edge: (edge["from"], edge["to"]))


def build_topology(positions_path, map_yaml_path, args):
    poses = load_positions(positions_path)
    if len(poses) < 2:
        raise RuntimeError("Need at least two poses to build topology")

    map_model = load_map_model(map_yaml_path, not args.allow_unknown)
    body_radius = math.hypot(args.robot_length * 0.5, args.robot_width * 0.5)
    required_clearance = (
        args.minimum_clearance
        if args.minimum_clearance is not None
        else body_radius + args.footprint_padding
    )
    if args.inflation_radius is not None:
        required_clearance = max(required_clearance, args.inflation_radius)
    if required_clearance < 0.0:
        raise ValueError("Minimum clearance must not be negative")

    node_clearances = []
    for pose in poses:
        clearance = point_clearance(pose, map_model)
        node_clearances.append(clearance)
        if clearance < 0.0:
            raise RuntimeError(f"P{pose['id']} is outside the static map")
        if clearance + 1e-9 < required_clearance:
            raise RuntimeError(
                f"P{pose['id']} clearance {clearance:.3f} m is below required "
                f"{required_clearance:.3f} m"
            )

    candidates = []
    rejected = Counter()
    for i in range(len(poses)):
        for j in range(i + 1, len(poses)):
            length = math.hypot(
                poses[i]["x"] - poses[j]["x"],
                poses[i]["y"] - poses[j]["y"],
            )
            if length < args.min_node_separation:
                raise RuntimeError(
                    f"P{i} and P{j} are only {length:.3f} m apart; merge duplicate points"
                )
            if length > args.max_pgm_edge_distance:
                rejected["too_far"] += 1
                continue
            result = evaluate_edge(
                poses[i], poses[j], map_model, required_clearance, args.sample_step
            )
            if not result["valid"]:
                rejected[result["reason"]] += 1
                continue
            candidates.append({
                "from": i,
                "to": j,
                "length": result["length"],
                "cost": edge_cost(
                    result["length"], result["min_clearance"], args.clearance_weight
                ),
                "min_clearance": result["min_clearance"],
                "trusted": False,
                "source": "map_validated",
                "bidirectional": True,
            })

    if not candidates:
        raise RuntimeError("No safe topology edge candidates were found")
    selected = select_sparse_edges(
        len(poses), candidates, args.min_node_degree, args.max_node_degree
    )
    components = graph_components(len(poses), selected)
    articulation, bridges = graph_cut_analysis(len(poses), selected)
    workstation_ids = [pose["id"] for pose in poses if pose["label"].startswith("W")]

    report = {
        "schema_version": 2,
        "mode": "audit" if args.audit else "build",
        "order_assumption": "unordered_nodes",
        "positions": os.path.abspath(positions_path),
        "map_yaml": map_model["yaml_path"],
        "map_image": map_model["pgm_path"],
        "positions_fingerprint": fingerprint_files([positions_path]),
        "map_fingerprint": fingerprint_files(
            [map_model["yaml_path"], map_model["pgm_path"]]
        ),
        "node_count": len(poses),
        "workstation_ids": workstation_ids,
        "map_occupied_cells": map_model["occupied_cells"],
        "map_unknown_cells": map_model["unknown_cells"],
        "unknown_is_obstacle": not args.allow_unknown,
        "required_clearance": required_clearance,
        "minimum_node_clearance": min(node_clearances),
        "candidate_edge_count": len(candidates),
        "selected_edge_count": len(selected),
        "rejected_edge_counts": dict(sorted(rejected.items())),
        "minimum_edge_clearance": min(edge["min_clearance"] for edge in selected),
        "component_count": len(components),
        "articulation_points": articulation,
        "bridges": [list(edge) for edge in bridges],
        "selected_edges": [
            {
                "from": edge["from"],
                "to": edge["to"],
                "length": round(edge["length"], 6),
                "cost": round(edge["cost"], 6),
                "min_clearance": round(edge["min_clearance"], 6),
            }
            for edge in selected
        ],
    }
    report["warnings"] = []
    if map_model["unknown_cells"] == 0:
        report["warnings"].append(
            "map has no unknown cells; unobserved space cannot be distinguished from known free space"
        )
    if articulation:
        report["warnings"].append(
            "topology still contains articulation points: "
            + ", ".join(f"P{node}" for node in articulation)
        )
    if bridges:
        report["warnings"].append(
            f"topology still contains {len(bridges)} bridge edge(s)"
        )
    return poses, selected, report


def topology_text(poses, edges, report, args):
    lines = [
        "# Generated by src/script/build_topology.py",
        "meta:",
        "  schema_version: 2",
        "  generation_mode: safety_sparse_unordered",
        f"  positions_fingerprint: {report['positions_fingerprint']}",
        f"  map_fingerprint: {report['map_fingerprint']}",
        f"  required_clearance: {report['required_clearance']:.6f}",
        f"  max_edge_distance: {args.max_pgm_edge_distance:.6f}",
        f"  unknown_is_obstacle: {'true' if report['unknown_is_obstacle'] else 'false'}",
        "  point_order_defines_edges: false",
        "nodes:",
    ]
    for pose in poses:
        lines.extend([
            f"  - id: {pose['id']}",
            f"    label: {json.dumps(pose['label'], ensure_ascii=False)}",
            f"    x: {pose['x']:.6f}",
            f"    y: {pose['y']:.6f}",
            f"    yaw: {pose['yaw']:.6f}",
        ])
    lines.append("edges:")
    for edge in edges:
        lines.extend([
            f"  - from: {edge['from']}",
            f"    to: {edge['to']}",
            f"    length: {edge['length']:.6f}",
            f"    cost: {edge['cost']:.6f}",
            f"    min_clearance: {edge['min_clearance']:.6f}",
            "    trusted: false",
            f"    source: {edge['source']}",
            f"    bidirectional: {'true' if edge['bidirectional'] else 'false'}",
            "    blocked: false",
        ])
    return "\n".join(lines) + "\n"


def atomic_write(path, content):
    directory = os.path.dirname(os.path.abspath(path))
    os.makedirs(directory, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w", encoding="utf-8", dir=directory,
            prefix=".topology.", suffix=".tmp", delete=False
        ) as handle:
            temporary = handle.name
            handle.write(content)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
        temporary = None
    finally:
        if temporary and os.path.exists(temporary):
            os.unlink(temporary)


def default_maps_dir():
    return "/home/nav/maps"


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Build/audit a safety-oriented topology from unordered waypoints."
    )
    parser.add_argument("--maps-dir", default=default_maps_dir())
    parser.add_argument("--positions", default=None)
    parser.add_argument("--map-yaml", default=None)
    parser.add_argument("--output", default=None)
    parser.add_argument("--report", default=None,
                        help="Optional JSON audit report path")
    parser.add_argument("--audit", action="store_true",
                        help="Analyze and print a report without replacing topology.yaml")
    parser.add_argument("--max-pgm-edge-distance", type=float, default=3.0)
    parser.add_argument("--minimum-clearance", type=float, default=None)
    parser.add_argument("--inflation-radius", type=float, default=None,
                        help="Deprecated compatibility floor for minimum clearance")
    parser.add_argument("--sample-step", type=float, default=0.05)
    parser.add_argument("--robot-length", type=float, default=0.72)
    parser.add_argument("--robot-width", type=float, default=0.50)
    parser.add_argument("--footprint-padding", type=float, default=0.15)
    parser.add_argument("--clearance-weight", type=float, default=0.30)
    parser.add_argument("--min-node-separation", type=float, default=0.10)
    parser.add_argument("--min-node-degree", type=int, default=2)
    parser.add_argument("--max-node-degree", type=int, default=4)
    parser.add_argument("--allow-unknown", action="store_true")
    args = parser.parse_args(argv)
    if args.max_pgm_edge_distance <= 0.0 or args.sample_step <= 0.0:
        parser.error("edge distance and sample step must be positive")
    if args.robot_length <= 0.0 or args.robot_width <= 0.0:
        parser.error("robot dimensions must be positive")
    if args.min_node_degree < 1 or args.max_node_degree < args.min_node_degree:
        parser.error("invalid min/max node degree")
    return args


def main(argv=None):
    args = parse_args(argv)
    positions_path = args.positions or os.path.join(args.maps_dir, "robot_positions.txt")
    map_yaml_path = args.map_yaml or os.path.join(args.maps_dir, "map.yaml")
    output_path = args.output or os.path.join(args.maps_dir, "topology.yaml")

    poses, edges, report = build_topology(positions_path, map_yaml_path, args)
    if args.report:
        atomic_write(args.report, json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    if not args.audit:
        atomic_write(output_path, topology_text(poses, edges, report, args))
        print(f"Wrote {output_path}")
    else:
        print("Audit only: topology.yaml was not modified")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    print(
        f"nodes={len(poses)} candidates={report['candidate_edge_count']} "
        f"edges={len(edges)} articulation={len(report['articulation_points'])} "
        f"bridges={len(report['bridges'])}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
