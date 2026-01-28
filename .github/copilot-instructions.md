# Copilot instructions for unitree_ws

Purpose: give AI coding agents the minimal, actionable knowledge to be productive in this mono-repo (ROS2 + native SDK).

Big picture
- Repo contains two primary code families: ROS2 bindings and native SDK.
  - ROS2-focused packages and examples under: [src/unitree_ros2](src/unitree_ros2/README.md)
  - Native C++ SDK and examples under: [src/unitree_sdk2](src/unitree_sdk2/README.md)
- Communication: DDS (CycloneDDS) is the middleware; ROS2 messages are generated in the cyclonedds workspace. Robot control flows use ROS2 topics (publish/sub) and a request/response sport API.

Key developer workflows (practical commands)
- Prepare environment (example for ROS2 Humble/Foxy):
  - `source /opt/ros/<distro>/setup.bash`
  - `source ~/unitree_ros2/setup.sh` (or `setup_local.sh` / `setup_default.sh` to use loopback)
- Build workspace:
  - If you need to build CycloneDDS first: `cd src/unitree_ros2/cyclonedds_ws && colcon build --packages-select cyclonedds`
  - Then from repo root: `colcon build`
- Run an example binary (after `colcon build`):
  - `./install/unitree_ros2_example/bin/read_motion_state`
  - `./install/unitree_ros2_example/bin/sport_mode_ctrl`

Project-specific conventions & patterns
- Middleware & network: the repo expects CycloneDDS as RMW. The workspace sets `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` and a `CYCLONEDDS_URI` that pins the network interface (see [src/unitree_ros2/setup.sh](src/unitree_ros2/setup.sh)).
- Topic naming: low-level/low-frequency topics often use `lf/` prefixes (e.g., `lf/sportmodestate`, `lf/lowstate`). Sport API uses `/api/sport/request` for request messages.
- Examples live under `example/` and install to `install/.../bin/` — use those binaries for quick runtime checks.

Integration points & files to inspect first
- Entry docs: [src/unitree_ros2/README.md](src/unitree_ros2/README.md) and [src/unitree_sdk2/README.md](src/unitree_sdk2/README.md).
- Setup scripts: [src/unitree_ros2/setup.sh](src/unitree_ros2/setup.sh), [src/unitree_ros2/setup_local.sh](src/unitree_ros2/setup_local.sh).
- Generated messages / cyclonedds workspace: [src/unitree_ros2/cyclonedds_ws](src/unitree_ros2/cyclonedds_ws)
- Examples (ROS2): [src/unitree_ros2/example](src/unitree_ros2/example)
- Native SDK build: [src/unitree_sdk2/CMakeLists.txt](src/unitree_sdk2/CMakeLists.txt) and `example/` subfolders in that package.

Debugging & gotchas (discovered from docs)
- Do NOT have ROS2 sourced when compiling CycloneDDS; it can break the cmake detection. If a build fails try `export LD_LIBRARY_PATH=/opt/ros/<distro>/lib` before building.
- If you need to run without a robot, use `setup_local.sh` to bind CycloneDDS to loopback (`lo`).

Actionable examples to reference in PRs or edits
- For adding new ROS2 interfaces: follow message/package layout inside `cyclonedds_ws/unitree/*` and regenerate with `colcon build`.
- For adding native examples: follow `src/unitree_sdk2/example/cmake_sample` pattern and update `CMAKE_PREFIX_PATH` if installing to custom prefix.

If anything above is unclear or you want more detail (e.g., exact message definitions, where to change CYCLONEDDS_URI, or common example source files), tell me which area to expand and I'll update this file.
