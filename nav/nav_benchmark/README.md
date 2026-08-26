# Navigation Benchmark

`nav_benchmark` is a read-only ROS1 logger for long-running Ranger navigation
acceptance tests. It does not publish goals or velocity commands and does not
modify planner/controller state.

## Automatic lifecycle

`robot_r/launch/3navlocations.launch` includes
`navigation_benchmark.launch`. Clicking **AutoNAV** in the GUI therefore starts
the logger automatically; **退出Nav** stops it and closes the session. The node
uses `respawn=true` and reopens an interrupted session after an unexpected
short process restart.

Runtime data is stored outside the source tree:

```text
~/maps/nav_benchmark_results/<session_id>/
├── session.json
├── runtime_state.json
├── samples.csv
├── events.csv
├── paths.csv
├── tasks.csv
└── live_summary.json
```

Rows are appended continuously. CSV streams are flushed for every row and
`fsync` is performed at least once per second and immediately for task/event
records.

## Measurement semantics

- A task starts/ends from `/anav/task_status` (`running`, `arrived`, `failed`,
  `canceled`).
- Arrival error is `map -> base_link` against the requested pose from
  `robot_positions.txt`. It is map-relative navigation error, not external
  physical docking ground truth.
- CTE is point-to-polyline distance against `/reference_path`, and is recorded
  only while `/bspline_status` is active, the selected command source is
  `nav`, and measured forward speed exceeds the tracking threshold.
- Start/final rotations and collision stops are not counted as normal B-spline
  tracking stops.
- Tag fine-positioning data is intentionally excluded.

## Offline summary

After the test, summarize the newest session:

```bash
rosrun nav_benchmark analyze_results.py
```

Or combine explicit fixed-route/reroute sessions:

```bash
rosrun nav_benchmark analyze_results.py SESSION_A SESSION_B \
  --output-dir ~/maps/nav_benchmark_results/combined
```

This writes `summary.json` and `summary.txt`.
