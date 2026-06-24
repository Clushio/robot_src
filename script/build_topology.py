#!/usr/bin/env python3
import argparse
import math
import os
from collections import deque


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

        if magic == b"P5":
            data = list(handle.read(width * height))
        else:
            rest = handle.read().split()
            data = [int(value) for value in rest[: width * height]]

    if len(data) != width * height:
        raise ValueError("PGM data size does not match width/height")
    if max_value <= 0:
        raise ValueError("Invalid PGM max value")
    return width, height, max_value, data


def load_positions(path):
    poses = []
    with open(path, "r", encoding="utf-8") as handle:
        for line_no, raw_line in enumerate(handle, 1):
            line = raw_line.strip()
            if not line:
                continue
            parts = line.split()
            if len(parts) < 6:
                raise ValueError(f"Invalid pose at line {line_no}: {line}")
            pose = {
                "id": len(poses),
                "x": float(parts[0]),
                "y": float(parts[1]),
                "z": float(parts[2]),
                "roll": float(parts[3]),
                "pitch": float(parts[4]),
                "yaw": float(parts[5]),
                "label": parts[6] if len(parts) > 6 else "",
            }
            poses.append(pose)
    return poses


def world_to_map(x, y, origin, resolution):
    mx = int(math.floor((x - origin[0]) / resolution))
    my = int(math.floor((y - origin[1]) / resolution))
    return mx, my


def is_occupied(pixel, max_value, occupied_thresh, negate):
    normalized = pixel / float(max_value)
    occupancy = normalized if negate else (1.0 - normalized)
    return occupancy >= occupied_thresh


def build_obstacle_grid(width, height, max_value, data, occupied_thresh, negate):
    return [
        is_occupied(pixel, max_value, occupied_thresh, negate)
        for pixel in data
    ]


def inflate_obstacles(grid, width, height, radius_cells):
    if radius_cells <= 0:
        return grid

    inflated = list(grid)
    distance = [-1] * (width * height)
    queue = deque()
    for index, occupied in enumerate(grid):
        if occupied:
            distance[index] = 0
            queue.append(index)

    neighbors = [(-1, -1), (0, -1), (1, -1),
                 (-1, 0),           (1, 0),
                 (-1, 1),  (0, 1),  (1, 1)]
    while queue:
        index = queue.popleft()
        current_distance = distance[index]
        if current_distance >= radius_cells:
            continue
        x = index % width
        y = index // width
        for dx, dy in neighbors:
            nx = x + dx
            ny = y + dy
            if nx < 0 or ny < 0 or nx >= width or ny >= height:
                continue
            next_index = ny * width + nx
            if distance[next_index] >= 0:
                continue
            distance[next_index] = current_distance + 1
            inflated[next_index] = True
            queue.append(next_index)
    return inflated


def segment_is_free(a, b, grid, width, height, origin, resolution, sample_step):
    distance = math.hypot(a["x"] - b["x"], a["y"] - b["y"])
    steps = max(1, int(math.ceil(distance / sample_step)))
    for step in range(steps + 1):
        t = step / float(steps)
        x = a["x"] + (b["x"] - a["x"]) * t
        y = a["y"] + (b["y"] - a["y"]) * t
        mx, my = world_to_map(x, y, origin, resolution)
        if mx < 0 or my < 0 or mx >= width or my >= height:
            return False
        if grid[my * width + mx]:
            return False
    return True


def edge_cost(a, b, source):
    cost = math.hypot(a["x"] - b["x"], a["y"] - b["y"])
    if source == "pgm":
        cost *= 1.10
    return cost


def add_edge(edges, seen, a, b, source, trusted, bidirectional=True):
    key = (min(a["id"], b["id"]), max(a["id"], b["id"]), source)
    if key in seen:
        return
    seen.add(key)
    edges.append({
        "from": a["id"],
        "to": b["id"],
        "cost": edge_cost(a, b, source),
        "trusted": trusted,
        "source": source,
        "bidirectional": bidirectional,
    })


def write_topology(path, poses, edges, args):
    with open(path, "w", encoding="utf-8") as handle:
        handle.write("# Generated by src/script/build_topology.py\n")
        handle.write("meta:\n")
        handle.write(f"  max_pgm_edge_distance: {args.max_pgm_edge_distance:.3f}\n")
        handle.write(f"  inflation_radius: {args.inflation_radius:.3f}\n")
        handle.write(f"  manual_edges_are_bidirectional: true\n")
        handle.write("nodes:\n")
        for pose in poses:
            label = pose["label"] if pose["label"] else ""
            handle.write(f"  - id: {pose['id']}\n")
            handle.write(f"    label: {label}\n")
            handle.write(f"    x: {pose['x']:.6f}\n")
            handle.write(f"    y: {pose['y']:.6f}\n")
            handle.write(f"    yaw: {pose['yaw']:.6f}\n")

        handle.write("edges:\n")
        for edge in edges:
            handle.write(f"  - from: {edge['from']}\n")
            handle.write(f"    to: {edge['to']}\n")
            handle.write(f"    cost: {edge['cost']:.6f}\n")
            handle.write(f"    trusted: {'true' if edge['trusted'] else 'false'}\n")
            handle.write(f"    source: {edge['source']}\n")
            handle.write(f"    bidirectional: {'true' if edge['bidirectional'] else 'false'}\n")
            handle.write("    blocked: false\n")


def default_maps_dir():
    return "/home/nav/maps"


def main():
    parser = argparse.ArgumentParser(description="Build waypoint topology from robot_positions and PGM map.")
    parser.add_argument("--maps-dir", default=default_maps_dir())
    parser.add_argument("--positions", default=None)
    parser.add_argument("--map-yaml", default=None)
    parser.add_argument("--output", default=None)
    parser.add_argument("--max-pgm-edge-distance", type=float, default=3.0)
    parser.add_argument("--inflation-radius", type=float, default=0.40)
    parser.add_argument("--sample-step", type=float, default=0.05)
    parser.add_argument("--no-pgm-edges", action="store_true")
    args = parser.parse_args()

    positions_path = args.positions or os.path.join(args.maps_dir, "robot_positions.txt")
    map_yaml_path = args.map_yaml or os.path.join(args.maps_dir, "map.yaml")
    output_path = args.output or os.path.join(args.maps_dir, "topology.yaml")

    poses = load_positions(positions_path)
    if len(poses) < 2:
        raise RuntimeError("Need at least two poses to build topology")

    edges = []
    seen = set()
    for index in range(len(poses) - 1):
        add_edge(edges, seen, poses[index], poses[index + 1], "manual", True)

    if not args.no_pgm_edges:
        map_info = read_simple_yaml(map_yaml_path)
        resolution = float(map_info["resolution"])
        origin = map_info.get("origin", [0.0, 0.0, 0.0])
        occupied_thresh = float(map_info.get("occupied_thresh", 0.65))
        negate = int(map_info.get("negate", 0))
        image = map_info.get("image", "map.pgm")
        pgm_path = image if os.path.isabs(image) else os.path.join(os.path.dirname(map_yaml_path), image)
        if os.path.isabs(image) and not os.path.exists(pgm_path):
            fallback = os.path.join(os.path.dirname(map_yaml_path), os.path.basename(image))
            if os.path.exists(fallback):
                pgm_path = fallback
        width, height, max_value, data = read_pgm(pgm_path)
        grid = build_obstacle_grid(width, height, max_value, data, occupied_thresh, negate)
        radius_cells = int(math.ceil(args.inflation_radius / resolution))
        inflated = inflate_obstacles(grid, width, height, radius_cells)

        for i in range(len(poses)):
            for j in range(i + 2, len(poses)):
                distance = math.hypot(poses[i]["x"] - poses[j]["x"], poses[i]["y"] - poses[j]["y"])
                if distance > args.max_pgm_edge_distance:
                    continue
                if segment_is_free(poses[i], poses[j], inflated, width, height, origin,
                                   resolution, args.sample_step):
                    add_edge(edges, seen, poses[i], poses[j], "pgm", False)

    write_topology(output_path, poses, edges, args)
    print(f"Wrote {output_path}")
    print(f"nodes={len(poses)} edges={len(edges)}")


if __name__ == "__main__":
    main()
