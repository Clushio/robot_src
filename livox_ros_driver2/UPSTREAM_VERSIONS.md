# Livox official source provenance

- `livox_ros_driver2`: <https://github.com/Livox-SDK/livox_ros_driver2.git>
  at `4a1def929e5b59c7a8122d19fce6efba581ce9f7` (2026-07-31,
  upstream version 1.2.7).
- `Livox-SDK2`: <https://github.com/Livox-SDK/Livox-SDK2.git>
  at `08f523c930b2f0ba1e98a6afaa8d7476bf479908` (2026-07-31).

The vendored SDK directory is an unmodified export of that official commit.
The ROS driver sources are also an official export; workspace-local changes are
limited to reproducible SDK integration, a normal-colcon default, standard ROS2
launch installation, and the active ROS2 package manifest. The upstream
blanket lint suite is not registered because it recursively rejects bundled
third-party sources; verification instead covers compilation, generated
interfaces, SDK linkage, package discovery and launch startup.
