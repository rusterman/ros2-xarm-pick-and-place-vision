# ros2-cheese-pick-place

A ROS2 robotic system for **detecting, picking, and placing cheese objects into a moving grid container** using computer vision and deep learning. Built with a fully containerized architecture that runs on **macOS (Apple Silicon / M chips)** out of the box.

> **Status:** Active development — infrastructure complete, xArm6 robot model integrated, vision pipeline and manipulation in progress.

---

## What This Project Does

A robotic arm observes a workspace through a camera. A deep learning model detects cheese objects and estimates their pose. The arm picks each piece and places it into the correct cell of a moving grid container on a conveyor belt — in real time.

```
Camera feed
    │
    ▼
Deep learning object detection (Python / OpenCV)
    │
    ▼
Pose estimation → ROS2 topic
    │
    ▼
Motion planner (C++) → joint trajectory
    │
    ▼
Robot arm execution (xArm6)
    │
    ▼
Cheese placed into moving grid container
```

---

## Technology Stack

| Layer | Technology |
|---|---|
| Robot middleware | ROS2 Humble |
| Robot hardware | UFACTORY xArm6 (6-DOF collaborative arm) |
| Visualization | Foxglove Studio (via WebSocket bridge) |
| Computer vision | OpenCV |
| Object detection | Deep learning (Python) |
| Motion planning | MoveIt2 + C++17 ROS2 nodes |
| Orchestration | Python ROS2 nodes |
| Container runtime | Docker (linux/arm64 — M chip native) |
| macOS bridge | Zenoh (TCP bridge over Docker port-forward) |

C++ is used for real-time, performance-critical nodes (motion planning, trajectory execution). Python is used for perception, deep learning inference, and high-level orchestration.

No Gazebo — the real xArm6 hardware is available from day one. Foxglove replaces the Gazebo GUI for visualization: robot model (URDF), TF frames, camera feed, and topic data are all visible in Foxglove over WebSocket.

---

## Architecture

### System Overview

```
macOS (Apple Silicon)
├── Foxglove Studio           ← connects via WebSocket ws://localhost:8765
├── ros2 CLI                  ← topic inspection, node management
├── zenoh-bridge (client)     ← bridges DDS over TCP
│       │
│       └── TCP localhost:7447
│
Docker container: ros2_dev    (linux/arm64 image)
├── foxglove_bridge           ← WebSocket server port 8765, bridges topics to Foxglove
├── zenoh-bridge (router)     ← port 7447
├── robot_state_publisher     ← publishes TF frames from xArm6 URDF
├── joint_state_publisher     ← publishes joint states
├── ROS2 nodes (FastDDS, domain 42)
│   ├── camera_node           ← publishes raw image stream
│   ├── detection_node        ← deep learning inference, publishes detections
│   ├── pose_estimator        ← 3D pose from 2D detections + depth
│   ├── motion_planner        ← C++ — computes pick/place trajectories
│   └── gripper_controller    ← C++ — executes joint commands
└── /ros2_ws/src/
```

### Why Docker + Zenoh on macOS

Docker Desktop on macOS runs containers inside a Linux VM. The container's bridge IP is unreachable from the host, so raw DDS multicast/unicast does not work across the boundary. Zenoh bridges ROS2 topics over TCP — which Docker port-forwarding supports cleanly.

Foxglove Studio connects to the container via `foxglove_bridge` over WebSocket — no X11 or display server required. The robot 3D model, TF tree, and all sensor topics are visible in Foxglove's panels.

### Current node graph

```
robot_state_publisher ──/tf, /tf_static──► Foxglove Studio (3D panel)
joint_state_publisher ──/joint_states───► robot_state_publisher
HelloPublisher        ──/hello_topic───► HelloSubscriber
MarkerPublisher       ──/visualization_marker──► Foxglove Studio
```

---

## Repository Structure

```
.
├── Dockerfile                  # ros:humble + colcon + zenoh-bridge + robot-state-publisher
├── docker-compose.yml          # container definition, ports 7447 and 8765
├── src/
│   ├── xarm_description/       # xArm6 URDF + mesh files (arm links)
│   │   ├── urdf/
│   │   │   └── xarm6_with_gripper.urdf
│   │   ├── meshes/xarm6/
│   │   │   ├── visual/         # STL files for Foxglove rendering
│   │   │   └── collision/      # OBJ files for collision detection
│   │   ├── launch/
│   │   │   └── display.launch.py
│   │   ├── package.xml
│   │   └── setup.py
│   ├── xarm_gripper/           # xArm gripper mesh files
│   │   ├── meshes/             # STL files for gripper fingers and knuckles
│   │   ├── package.xml
│   │   └── setup.py
│   ├── hello_ros2/             # Python scaffold package (publisher, subscriber, marker)
│   │   ├── hello_ros2/
│   │   │   ├── publisher.py
│   │   │   ├── subscriber.py
│   │   │   └── marker_publisher.py
│   │   ├── package.xml
│   │   └── setup.py
│   └── hello_cpp/              # C++ scaffold package (publisher, subscriber)
│       ├── src/
│       │   ├── publisher.cpp
│       │   └── subscriber.cpp
│       ├── package.xml
│       └── CMakeLists.txt
├── xarm/                       # Original URDF files provided by UFACTORY (reference only)
├── config/
│   └── cyclonedds_macos.xml    # CycloneDDS loopback config (AllowMulticast=spdp)
├── bin/                        # gitignored — populated by download_zenoh.sh
│   └── zenoh-bridge-ros2dds
├── scripts/
│   ├── download_zenoh.sh       # downloads macOS Zenoh bridge binary (run once)
│   ├── start_bridge.sh         # starts Docker router + macOS client bridge
│   ├── ros2.sh                 # ros2 CLI wrapper (fixes Homebrew Python conflict)
└── docs/
    ├── DESIGN.md
    ├── ros2-pubsub-foxglove-architecture.md
    ├── cyclonedds_macos.md
    ├── rmw-ros2-middleware.md
    ├── ros2-package-files.md
    ├── scripts.md
    └── colcon-build-modules-package-xml.md
```

---

## Prerequisites

- **Docker Desktop for Mac** (Apple Silicon build) — [download](https://www.docker.com/products/docker-desktop/)
- **Miniforge** with a `ros2` conda environment (RoboStack, ROS2 Humble)
  - The `ros2` CLI runs here natively on macOS
- **Foxglove Studio** — [download](https://foxglove.dev/download) — native macOS app, connects via WebSocket

---

## Quick Start

```bash
# 1. Clone
git clone https://github.com/rusterman/ros2-cheese-pick-place.git
cd ros2-cheese-pick-place

# 2. Download the macOS Zenoh bridge binary (gitignored, run once)
./scripts/download_zenoh.sh

# 3. Build Docker image
docker compose build

# 4. Start container and build ROS2 workspace
docker compose up -d
docker exec ros2_dev bash -c "
  source /opt/ros/humble/setup.bash &&
  cd /ros2_ws &&
  colcon build --symlink-install"
```

---

## Visualizing the xArm6 Robot in Foxglove

```bash
# Terminal 1 — start the container (if not already running)
docker compose up -d

# Terminal 2 — build the xarm packages (first time only)
docker exec -it ros2_dev bash -c "
  cd /ros2_ws &&
  colcon build --packages-select xarm_description xarm_gripper"

# Terminal 3 — start foxglove_bridge
docker exec -it ros2_dev bash -c "
  source /opt/ros/humble/setup.bash &&
  ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765"

# Terminal 4 — launch the robot state publisher
docker exec -it ros2_dev bash -c "
  source /opt/ros/humble/setup.bash &&
  source /ros2_ws/install/setup.bash &&
  ros2 launch xarm_description display.launch.py"

# Open Foxglove Studio
open /Applications/Foxglove.app
# Connect: Open Connection → Foxglove WebSocket → ws://localhost:8765
# Add panel → 3D → subscribe to /tf and /tf_static
# The xArm6 with gripper will render in its default (zero-angles) pose
```

---

## Running the Current Scaffold

```bash
# Terminal 1 — start the container
docker compose up -d

# Terminal 2 — start Zenoh bridge (keep open, needed for ros2 CLI on macOS)
./scripts/start_bridge.sh

# Terminal 3 — start foxglove_bridge (keep open, needed for Foxglove Studio)
docker exec -it ros2_dev bash -c "
  source /opt/ros/humble/setup.bash &&
  ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765"

# Terminal 4 — Python publisher node
docker exec -it ros2_dev bash -c "
  source /opt/ros/humble/setup.bash &&
  source /ros2_ws/install/setup.bash &&
  ros2 run hello_ros2 publisher"

# Terminal 5 — Python subscriber node
docker exec -it ros2_dev bash -c "
  source /opt/ros/humble/setup.bash &&
  source /ros2_ws/install/setup.bash &&
  ros2 run hello_ros2 subscriber"

# Terminal 6 — marker publisher (visible in Foxglove)
docker exec -it ros2_dev bash -c "
  source /opt/ros/humble/setup.bash &&
  source /ros2_ws/install/setup.bash &&
  ros2 run hello_ros2 marker_publisher"

# Terminal 7 — C++ publisher node
docker exec -it ros2_dev bash -c "
  source /opt/ros/humble/setup.bash &&
  source /ros2_ws/install/setup.bash &&
  ros2 run hello_cpp publisher"

# Terminal 8 — C++ subscriber node
docker exec -it ros2_dev bash -c "
  source /opt/ros/humble/setup.bash &&
  source /ros2_ws/install/setup.bash &&
  ros2 run hello_cpp subscriber"

# Open Foxglove Studio (native macOS app)
open /Applications/Foxglove.app
# In Foxglove: Open Connection → Foxglove WebSocket → ws://localhost:8765 → Open
# IMPORTANT: select "Foxglove WebSocket", NOT "Rosbridge WebSocket"
# Add panel → 3D → Fixed Frame = "map" → subscribe to /visualization_marker
```

### Inspect topics from macOS

```bash
./scripts/ros2.sh topic list
./scripts/ros2.sh topic echo /hello_topic
./scripts/ros2.sh topic echo /visualization_marker
./scripts/ros2.sh node list
```

---

## Stop

```bash
# Ctrl+C in each docker exec terminal to stop nodes
# Ctrl+C in start_bridge.sh terminal to stop bridge
docker compose down
```

---

## Known Issues and Fixes

| Issue | Fix applied |
|---|---|
| Container bridge IP unreachable from macOS | Zenoh bridge over TCP port 7447 |
| Homebrew Python 3.14 breaks rclpy imports | `scripts/ros2.sh` unsets `PYTHONPATH`, uses conda Python directly |
| `ros2 topic list` hangs 60 s | `--no-daemon` injected by wrapper for verbs that support it |
| `ros2 topic hz` rejected `--no-daemon` | Wrapper applies `--no-daemon` only to compatible verbs |
| CycloneDDS `<Interfaces>` + `ROS_LOCALHOST_ONLY` conflict | Interface selection via `CYCLONEDDS_URI`; `ROS_LOCALHOST_ONLY` not used |
| Message triplication on macOS loopback | `AllowMulticast=spdp` in `cyclonedds_macos.xml` — data uses unicast only |
| Missing Docker zenoh router step | `start_bridge.sh` starts Docker router first, then macOS client |

---

## Roadmap

- [x] URDF robot model integrated (xArm6 + gripper, visualized in Foxglove)
- [ ] Camera node (simulated + real)
- [ ] Deep learning cheese detection node (Python)
- [ ] 3D pose estimation from depth image
- [ ] C++ motion planning node (MoveIt2)
- [ ] Gripper controller node (C++)
- [ ] Moving grid container tracking (conveyor belt)
- [ ] End-to-end pick and place demo
- [ ] Foxglove Studio dashboard (detections, trajectory preview, state machine monitor)

---

## License

MIT
