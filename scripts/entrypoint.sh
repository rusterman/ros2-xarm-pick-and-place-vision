#!/usr/bin/env bash
# Container entrypoint: set up the ROS2 environment, build the workspace,
# then hand off to whatever command the container was started with.
set -e

# 1. Base ROS2 environment.
source /opt/ros/humble/setup.bash

# 2. Build your mounted packages. --symlink-install + no C++ = fast, and colcon
#    skips unchanged packages. Don't kill the shell if a build error happens.
cd /ros2_ws
colcon build --symlink-install || echo "[entrypoint] colcon build failed — dropping to shell anyway"

# 3. Overlay the freshly built workspace.
[ -f /ros2_ws/install/setup.bash ] && source /ros2_ws/install/setup.bash

# 4. Run whatever was passed (defaults to CMD = bash).
exec "$@"