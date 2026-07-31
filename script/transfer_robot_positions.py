#!/usr/bin/env python3
"""Transfer ROS robot positions between two independently built 2-D maps.

The map YAML files provide image resolution and origin, while feature matching
between their PGM images supplies the missing rotation and translation between
the two map frames.
"""

import argparse
import math
import sys
from pathlib import Path

import cv2
import numpy as np


DEFAULT_TRANSFER_DIR = Path.home() / "maps" / "transfer"
DEFAULT_OLD_MAP_YAML = DEFAULT_TRANSFER_DIR / "map_old" / "map.yaml"
DEFAULT_NEW_MAP_YAML = DEFAULT_TRANSFER_DIR / "map_new" / "map.yaml"
DEFAULT_POSITIONS = DEFAULT_TRANSFER_DIR / "robot_positions.txt"
DEFAULT_POSITIONS_OUTPUT = (
    DEFAULT_TRANSFER_DIR / "robot_positions_transformed.txt"
)


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
                items = [
                    item.strip()
                    for item in value[1:-1].split(",")
                    if item.strip()
                ]
                values[key] = [float(item) for item in items]
            else:
                try:
                    values[key] = float(value)
                except ValueError:
                    values[key] = value.strip("'\"")
    return values


def resolve_map_image(yaml_path, image_value):
    yaml_path = Path(yaml_path).resolve()
    image_path = Path(str(image_value)).expanduser()
    if not image_path.is_absolute():
        image_path = yaml_path.parent / image_path
    if image_path.is_file():
        return image_path.resolve()

    # Saved map YAML files are often moved together with their PGM while the
    # absolute image entry still points at the old directory.
    sibling = yaml_path.parent / image_path.name
    if sibling.is_file():
        print(
            "warning: YAML image path does not exist; using sibling "
            f"{sibling}",
            file=sys.stderr,
        )
        return sibling.resolve()
    raise FileNotFoundError(
        f"map image not found: YAML says {image_value!r}, also tried {sibling}"
    )


def load_map(yaml_path):
    values = read_simple_yaml(yaml_path)
    if "resolution" not in values or "origin" not in values:
        raise ValueError(f"{yaml_path}: missing resolution or origin")
    resolution = float(values["resolution"])
    origin = values["origin"]
    if resolution <= 0 or not isinstance(origin, list) or len(origin) < 2:
        raise ValueError(f"{yaml_path}: invalid resolution/origin")
    origin = list(origin)
    if len(origin) < 3:
        origin.append(0.0)

    image_path = resolve_map_image(yaml_path, values.get("image", "map.pgm"))
    image = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise ValueError(f"unable to read map image: {image_path}")

    negate = int(values.get("negate", 0))
    occupied_thresh = float(values.get("occupied_thresh", 0.65))
    normalized = image.astype(np.float32) / 255.0
    occupancy = normalized if negate else 1.0 - normalized
    occupied = np.uint8(occupancy >= occupied_thresh) * 255
    return {
        "yaml": Path(yaml_path).resolve(),
        "image_path": image_path,
        "image": image,
        "occupied": occupied,
        "resolution": resolution,
        "origin": np.array(origin[:3], dtype=np.float64),
        "height": image.shape[0],
        "width": image.shape[1],
    }


def pixels_to_world(points, map_info):
    points = np.asarray(points, dtype=np.float64)
    local = np.empty_like(points)
    local[:, 0] = points[:, 0] * map_info["resolution"]
    local[:, 1] = (
        map_info["height"] - 1 - points[:, 1]
    ) * map_info["resolution"]
    yaw = map_info["origin"][2]
    rotation = np.array(
        [
            [math.cos(yaw), -math.sin(yaw)],
            [math.sin(yaw), math.cos(yaw)],
        ]
    )
    return local.dot(rotation.T) + map_info["origin"][:2]


def world_to_map_local(point, map_info):
    offset = np.asarray(point, dtype=np.float64) - map_info["origin"][:2]
    yaw = map_info["origin"][2]
    inverse_rotation = np.array(
        [
            [math.cos(yaw), math.sin(yaw)],
            [-math.sin(yaw), math.cos(yaw)],
        ]
    )
    return inverse_rotation.dot(offset)


def pose_is_in_map(pose, map_info):
    local = world_to_map_local(pose[:2], map_info)
    return (
        0.0 <= local[0] < map_info["width"] * map_info["resolution"]
        and 0.0 <= local[1] < map_info["height"] * map_info["resolution"]
    )


def fit_rigid_transform(source, target):
    source_center = np.mean(source, axis=0)
    target_center = np.mean(target, axis=0)
    centered_source = source - source_center
    centered_target = target - target_center
    u, _singular, vt = np.linalg.svd(centered_source.T.dot(centered_target))
    rotation = vt.T.dot(u.T)
    if np.linalg.det(rotation) < 0:
        vt[-1, :] *= -1
        rotation = vt.T.dot(u.T)
    translation = target_center - rotation.dot(source_center)
    return rotation, translation


def estimate_map_transform(old_map, new_map, args):
    detector = cv2.AKAZE_create(
        descriptor_type=cv2.AKAZE_DESCRIPTOR_MLDB,
        threshold=args.feature_threshold,
    )
    old_keypoints, old_descriptors = detector.detectAndCompute(
        old_map["occupied"], None
    )
    new_keypoints, new_descriptors = detector.detectAndCompute(
        new_map["occupied"], None
    )
    if old_descriptors is None or new_descriptors is None:
        raise RuntimeError("not enough map features for registration")

    matches = cv2.BFMatcher(cv2.NORM_HAMMING).knnMatch(
        old_descriptors, new_descriptors, k=2
    )
    good_matches = []
    for candidates in matches:
        if len(candidates) < 2:
            continue
        first, second = candidates
        if first.distance < args.ratio_test * second.distance:
            good_matches.append(first)
    if len(good_matches) < args.min_matches:
        raise RuntimeError(
            f"only {len(good_matches)} feature matches; need "
            f"at least {args.min_matches}"
        )

    old_pixels = np.float32(
        [old_keypoints[match.queryIdx].pt for match in good_matches]
    )
    new_pixels = np.float32(
        [new_keypoints[match.trainIdx].pt for match in good_matches]
    )
    old_world = pixels_to_world(old_pixels, old_map)
    new_world = pixels_to_world(new_pixels, new_map)

    similarity, ransac_inliers = cv2.estimateAffinePartial2D(
        old_world,
        new_world,
        method=cv2.RANSAC,
        ransacReprojThreshold=args.ransac_error,
        maxIters=20000,
        confidence=0.999,
        refineIters=50,
    )
    if similarity is None or ransac_inliers is None:
        raise RuntimeError("map registration failed")
    selected = ransac_inliers.ravel().astype(bool)
    if int(np.count_nonzero(selected)) < args.min_inliers:
        raise RuntimeError(
            f"only {np.count_nonzero(selected)} registration inliers; need "
            f"at least {args.min_inliers}"
        )

    similarity_scale = math.hypot(similarity[0, 0], similarity[1, 0])
    if abs(similarity_scale - 1.0) > args.max_scale_error:
        raise RuntimeError(
            "map scale disagreement is too large: "
            f"{similarity_scale:.6f}; limit is {args.max_scale_error:.6f}"
        )

    # Topology coordinates represent physical metres, so use an SE(2) rigid
    # transform. The similarity scale remains a useful registration diagnostic.
    rotation, translation = fit_rigid_transform(
        old_world[selected], new_world[selected]
    )
    predictions = old_world.dot(rotation.T) + translation
    errors = np.linalg.norm(predictions - new_world, axis=1)

    # One refinement removes borderline RANSAC matches before the final fit.
    refined = selected & (errors <= args.ransac_error)
    if int(np.count_nonzero(refined)) >= args.min_inliers:
        rotation, translation = fit_rigid_transform(
            old_world[refined], new_world[refined]
        )
        selected = refined
        predictions = old_world.dot(rotation.T) + translation
        errors = np.linalg.norm(predictions - new_world, axis=1)

    selected_errors = errors[selected]
    p95_error = float(np.percentile(selected_errors, 95))
    if p95_error > args.max_p95_error:
        raise RuntimeError(
            f"registration p95 error {p95_error:.4f} m exceeds "
            f"limit {args.max_p95_error:.4f} m"
        )

    angle = math.atan2(rotation[1, 0], rotation[0, 0])
    stats = {
        "old_features": len(old_keypoints),
        "new_features": len(new_keypoints),
        "matches": len(good_matches),
        "inliers": int(np.count_nonzero(selected)),
        "similarity_scale": similarity_scale,
        "median_error": float(np.median(selected_errors)),
        "p95_error": p95_error,
        "max_error": float(np.max(selected_errors)),
        "angle": angle,
    }
    return rotation, translation, stats


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def transform_pose(x, y, yaw, rotation, translation, angle):
    transformed = rotation.dot(np.array([x, y])) + translation
    return (
        float(transformed[0]),
        float(transformed[1]),
        normalize_angle(yaw + angle),
    )


def read_position_poses(input_path):
    poses = []
    with open(input_path, "r", encoding="utf-8") as handle:
        for line_number, raw_line in enumerate(handle, 1):
            stripped = raw_line.strip()
            if not stripped or stripped.startswith("#"):
                continue
            parts = stripped.split()
            if len(parts) < 6:
                raise ValueError(
                    f"{input_path}:{line_number}: expected at least 6 fields"
                )
            poses.append(
                (float(parts[0]), float(parts[1]), float(parts[5]))
            )
    return poses


def validate_pose_bounds(
    poses, map_info, description, allow_outside, rotation=None,
    translation=None, angle=0.0
):
    checked_poses = poses
    if rotation is not None:
        checked_poses = [
            transform_pose(
                pose[0],
                pose[1],
                pose[2],
                rotation,
                translation,
                angle,
            )
            for pose in poses
        ]
    outside = [
        (index, pose)
        for index, pose in enumerate(checked_poses)
        if not pose_is_in_map(pose, map_info)
    ]
    if not outside:
        return
    samples = ", ".join(
        f"#{index} ({pose[0]:.3f}, {pose[1]:.3f})"
        for index, pose in outside[:3]
    )
    message = (
        f"{len(outside)}/{len(checked_poses)} {description} points are outside "
        f"{map_info['yaml']}; examples: {samples}"
    )
    if not allow_outside:
        raise ValueError(message + "; check map direction/files or pass "
                         "--allow-outside-map")
    print(f"warning: {message}", file=sys.stderr)


def ensure_writable_output(path, force):
    path = Path(path)
    if path.exists() and not force:
        raise FileExistsError(
            f"output already exists: {path}; pass --force to replace it"
        )
    path.parent.mkdir(parents=True, exist_ok=True)
    return path


def transform_positions(
    input_path, output_path, rotation, translation, angle, force
):
    output_path = ensure_writable_output(output_path, force)
    output_lines = []
    count = 0
    with open(input_path, "r", encoding="utf-8") as handle:
        for line_number, raw_line in enumerate(handle, 1):
            stripped = raw_line.strip()
            if not stripped or stripped.startswith("#"):
                output_lines.append(raw_line)
                continue
            parts = stripped.split()
            if len(parts) < 6:
                raise ValueError(
                    f"{input_path}:{line_number}: expected at least 6 fields"
                )
            x, y, yaw = float(parts[0]), float(parts[1]), float(parts[5])
            x, y, yaw = transform_pose(
                x, y, yaw, rotation, translation, angle
            )
            parts[0] = f"{x:.6f}"
            parts[1] = f"{y:.6f}"
            parts[5] = f"{yaw:.6f}"
            output_lines.append(" ".join(parts) + "\n")
            count += 1

    with open(output_path, "w", encoding="utf-8") as handle:
        handle.writelines(output_lines)
    return count, output_path


def build_parser():
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
        description=(
            "Register map_old/map.yaml and map_new/map.yaml, then transfer "
            "robot_positions.txt into the new map frame."
        )
    )
    parser.add_argument(
        "--old-map-yaml",
        default=str(DEFAULT_OLD_MAP_YAML),
    )
    parser.add_argument(
        "--new-map-yaml",
        default=str(DEFAULT_NEW_MAP_YAML),
    )
    parser.add_argument(
        "--positions",
        default=str(DEFAULT_POSITIONS),
    )
    parser.add_argument(
        "--positions-output",
        default=str(DEFAULT_POSITIONS_OUTPUT),
    )
    parser.add_argument("--force", action="store_true")
    parser.add_argument(
        "--allow-outside-map",
        action="store_true",
        help="allow source/destination position points outside map bounds",
    )
    parser.add_argument("--feature-threshold", type=float, default=0.0001)
    parser.add_argument("--ratio-test", type=float, default=0.75)
    parser.add_argument("--min-matches", type=int, default=20)
    parser.add_argument("--min-inliers", type=int, default=12)
    parser.add_argument("--ransac-error", type=float, default=0.03)
    parser.add_argument("--max-p95-error", type=float, default=0.05)
    parser.add_argument("--max-scale-error", type=float, default=0.03)
    return parser


def main():
    args = build_parser().parse_args()

    old_map = load_map(args.old_map_yaml)
    new_map = load_map(args.new_map_yaml)
    rotation, translation, stats = estimate_map_transform(
        old_map, new_map, args
    )

    poses = read_position_poses(args.positions)
    if not poses:
        raise ValueError("robot_positions.txt contains no points")
    validate_pose_bounds(
        poses,
        old_map,
        "source position",
        args.allow_outside_map,
    )
    validate_pose_bounds(
        poses,
        new_map,
        "transformed position",
        args.allow_outside_map,
        rotation,
        translation,
        stats["angle"],
    )

    print(f"old map: {old_map['yaml']} -> {old_map['image_path']}")
    print(f"new map: {new_map['yaml']} -> {new_map['image_path']}")
    print(
        "registration: "
        f"features={stats['old_features']}/{stats['new_features']} "
        f"matches={stats['matches']} inliers={stats['inliers']}"
    )
    print(
        "quality: "
        f"median={stats['median_error']:.4f} m "
        f"p95={stats['p95_error']:.4f} m "
        f"max={stats['max_error']:.4f} m "
        f"similarity_scale={stats['similarity_scale']:.6f}"
    )
    print(
        "old -> new SE(2): "
        f"rotation={math.degrees(stats['angle']):.6f} deg "
        f"translation=({translation[0]:.6f}, {translation[1]:.6f}) m"
    )

    count, output = transform_positions(
        args.positions,
        args.positions_output,
        rotation,
        translation,
        stats["angle"],
        args.force,
    )
    print(f"wrote {count} positions: {output}")


if __name__ == "__main__":
    try:
        main()
    except (FileNotFoundError, FileExistsError, RuntimeError, ValueError) as error:
        print(f"error: {error}", file=sys.stderr)
        sys.exit(2)
