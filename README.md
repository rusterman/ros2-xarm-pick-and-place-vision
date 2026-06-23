# ros2-cheese-pick-place

A ROS2 robotic system for **detecting, picking, and placing cheese slices into a moving grid container** using computer vision and motion planning. Built with a fully containerized architecture that runs on **macOS (Apple Silicon / M chips)** out of the box.

> **Status:** Active development — infrastructure complete, xArm6 robot model integrated and animated in Foxglove. Gazebo Classic 11 simulation (amd64 container) now renders a real camera feed of the arm into Foxglove and a real Gazebo GUI into the browser via noVNC. Perception pipeline and manipulation in progress.

---

## What This Project Does

A conveyor belt carries cheese slices at constant speed. A camera detects each slice and estimates its 3D pose. The robot arm picks each piece and places it into the correct grid slot of a moving container — in real time, without stopping the belt.

```
Orbbec Astra camera
    │
    ▼
Point cloud processing (PCL — geometric detection, no AI)
    │
    ▼
Conveyor state estimator (position tracking via belt speed)
    │
    ▼
Task scheduler (feasibility check, deadline-based queue)
    │
    ▼
MoveIt2 motion planner (IK + collision-aware trajectories)
    │
    ▼
xArm6 robot arm + mechanical gripper
    │
    ▼
Cheese placed into correct grid slot
```

---

## Technology Stack

| Layer | Technology |
|---|---|
| Robot middleware | ROS2 Humble |
| Robot hardware | UFACTORY xArm6 (6-DOF collaborative arm) |
| Gripper | Mechanical parallel gripper with widened fingertips |
| Camera | Orbbec Astra Series (structured light, RGB-D) |
| Visualization | Foxglove Studio (via WebSocket bridge) |
| Computer vision | OpenCV + PCL (geometric detection, no AI/ML) |
| Motion planning | MoveIt2 |
| Container runtime | Docker (linux/amd64 — emulated on Apple Silicon, required for Gazebo Classic's apt dependency chain) |
| Simulation | Gazebo Classic 11 (`gazebo_ros_pkgs`) — headless, Xvfb + Mesa software rendering |
| Remote GUI | noVNC (`x11vnc` + `websockify`) — view the real Gazebo GUI in a browser |
| macOS bridge | Zenoh (TCP bridge over Docker port-forward) |

**No AI/ML.** The real xArm6 hardware is available from day one. Gazebo Classic provides a real rendered camera feed (not synthetic) for visualization in Foxglove and direct GUI access via noVNC; object detection uses geometric PCL processing — sufficient for geometrically consistent cheese slices within a batch.

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
Docker container: ros2_dev    (linux/arm64)
├── foxglove_bridge           ← WebSocket server :8765
├── zenoh-bridge (router)     ← port 7447
├── robot_state_publisher     ← publishes TF from xArm6 URDF
├── arm_demo                  ← animated joint state publisher (demo)
└── /ros2_ws/src/
```

### Current Node Graph

```
arm_demo ──/joint_states──► robot_state_publisher ──/tf──► Foxglove 3D panel
foxglove_bridge ──ws:8765──► Foxglove Studio
```

### Why Docker + Zenoh on macOS

Docker Desktop on macOS runs containers inside a Linux VM. The container's bridge IP is unreachable from the host, so raw DDS does not work across the boundary. Zenoh bridges ROS2 topics over TCP — which Docker port-forwarding supports cleanly. Foxglove connects via WebSocket — no X11 or display server required.

---

## Repository Structure

```
.
├── Dockerfile                  # ros:humble + colcon + zenoh-bridge + robot-state-publisher
├── docker-compose.yml          # ports 7447 (zenoh) and 8765 (foxglove)
├── src/
│   ├── xarm_description/       # xArm6 URDF, meshes, launch, demo nodes
│   │   ├── urdf/
│   │   │   └── xarm6_with_gripper.urdf
│   │   ├── meshes/xarm6/
│   │   │   ├── visual/         # STL files for Foxglove rendering
│   │   │   └── collision/      # OBJ files for collision detection
│   │   ├── launch/
│   │   │   └── display.launch.py
│   │   ├── xarm_description/
│   │   │   ├── arm_demo.py     # animated demo — rotates arm, opens/closes gripper
│   │   │   └── joint_state_pub.py  # static hold-up pose publisher
│   │   ├── package.xml
│   │   └── setup.py
│   ├── xarm_gripper/           # xArm gripper STL meshes
│   │   ├── meshes/             # fingers, knuckles
│   │   ├── package.xml
│   │   └── setup.py
│   ├── hello_ros2/             # Python scaffold (publisher, subscriber, marker)
│   └── hello_cpp/              # C++ scaffold (publisher, subscriber)
├── xarm/                       # Original URDF files from UFACTORY (reference, not used in build)
├── config/
│   └── cyclonedds_macos.xml    # CycloneDDS loopback config
├── bin/                        # gitignored — populated by download_zenoh.sh
│   └── zenoh-bridge-ros2dds
├── scripts/
│   ├── download_zenoh.sh       # downloads macOS Zenoh bridge binary (run once)
│   ├── start_bridge.sh         # starts Docker router + macOS client bridge
│   └── ros2.sh                 # ros2 CLI wrapper (fixes Homebrew Python conflict)
└── docs/
    ├── ARCHITECTURE.md         # full system architecture document
    ├── GAZEBO_SIMULATION.md    # Gazebo install/launch logic, diagrams, and why each choice was made
    └── ...
```

---

## Prerequisites

- **Docker Desktop for Mac** (Apple Silicon) — [download](https://www.docker.com/products/docker-desktop/)
- **Foxglove Studio** — [download](https://foxglove.dev/download)
- **Miniforge** with a `ros2` conda environment (RoboStack, ROS2 Humble) — for macOS CLI only

---

## Quick Start

```bash
# 1. Clone
git clone https://github.com/rusterman/ros2-cheese-pick-place.git
cd ros2-cheese-pick-place

# 2. Download the macOS Zenoh bridge binary (run once)
./scripts/download_zenoh.sh

# 3. Build and start container
docker compose build
docker compose up -d
```

---

## Visualizing the xArm6 in Foxglove (Animated Demo)

Launch order matters — start each in a separate terminal and keep them running.

```bash
# Terminal 1 — build xarm packages (first time or after code changes)
docker exec ros2_dev bash -c "
  source /opt/ros/humble/setup.bash &&
  cd /ros2_ws &&
  colcon build --packages-select xarm_description xarm_gripper"

# Terminal 2 — robot state publisher (computes TF from joint states + URDF)
docker exec -it ros2_dev bash -c "
  source /opt/ros/humble/setup.bash &&
  source /ros2_ws/install/setup.bash &&
  ros2 launch xarm_description display.launch.py"

# Terminal 3 — foxglove bridge (WebSocket server for Foxglove Studio)
docker exec -it ros2_dev bash -c "
  source /opt/ros/humble/setup.bash &&
  source /ros2_ws/install/setup.bash &&
  ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765"

# Terminal 4 — arm demo (animated joint states: arm rotation + gripper open/close)
docker exec -it ros2_dev bash -c "
  source /opt/ros/humble/setup.bash &&
  source /ros2_ws/install/setup.bash &&
  ros2 run xarm_description arm_demo"
```

### Foxglove Studio Setup

1. Open Foxglove Studio
2. **Open Connection** → **Foxglove WebSocket** → `ws://localhost:8765` → **Open**
3. Set **Display frame** → `world`
4. **Custom layers** → **+** → **URDF**
   - URL: `package://xarm_description/urdf/xarm6_with_gripper.urdf`
   - Control mode: `Transforms`
5. **Scene** → **Mesh "up" axis** → `Y-up`

The xArm6 with gripper will appear and animate — rotating through 0° / 90° / 180° / -90° while opening and closing the gripper.

> **Note:** The Mesh "up" axis must be set to **Y-up** for the UFACTORY STL meshes to render correctly. Z-up causes the links to appear disconnected.

---

## Static Pose (No Animation)

To display the arm in a fixed hold-up pose without animation:

```bash
# Replace Terminal 4 with:
docker exec -it ros2_dev bash -c "
  source /opt/ros/humble/setup.bash &&
  source /ros2_ws/install/setup.bash &&
  ros2 run xarm_description joint_state_pub"
```

---

## Gazebo Classic Simulation (Real Rendered Camera + Browser GUI)

Runs the xArm6 + Orbbec camera inside a real physics/rendering simulation (Gazebo Classic 11), with the camera's actual rendered output flowing into Foxglove, and a full Linux desktop (fluxbox window manager + the Gazebo GUI, auto-maximized) viewable in a browser tab via noVNC. The container must be built for `linux/amd64` (already set in `docker-compose.yml`) — Gazebo's apt dependency chain is broken on arm64.

> See [docs/GAZEBO_SIMULATION.md](docs/GAZEBO_SIMULATION.md) for the architecture diagram, launch sequencing logic, and the reasoning behind each choice below.

```bash
# 1. Build and start the container (first time, or after editing the Dockerfile)
docker compose build
docker compose up -d

# 2. Build the workspace packages (first time, or after editing URDF/world/launch files)
docker exec ros2_dev bash -c "
  source /opt/ros/humble/setup.bash &&
  cd /ros2_ws &&
  colcon build --symlink-install --packages-select xarm_description xarm_gripper orbbec_description"

# 3. Launch everything — Xvfb, gzserver, robot_state_publisher, joint_state_pub,
#    spawn the robot, foxglove_bridge, and the noVNC stack (x11vnc + websockify + gzclient)
docker exec -it ros2_dev bash -c "
  source /opt/ros/humble/setup.bash &&
  source /ros2_ws/install/setup.bash &&
  ros2 launch xarm_description gazebo_camera_test.launch.py"
```

That's it — one launch command brings up the full simulation. Ctrl+C stops everything cleanly.

### Viewing the results

- **Foxglove Studio** → Open Connection → `ws://localhost:8765`
  - **Image panel** → `/camera/color/image_raw` — the real Gazebo-rendered camera feed
  - **3D panel** → robot model + `/camera/depth/points`
- **Browser (noVNC)** → `http://localhost:6080/vnc.html?resize=scale` → Connect (no password) — a full desktop (fluxbox) with the Gazebo GUI already maximized
  - The `?resize=scale` stretches the view to fill your browser window client-side (free — no extra server-side rendering cost). You can also toggle this manually via the gear/Settings icon in the noVNC sidebar → **Scaling Mode** → **Local Scaling**.
  - If you see `noVNC encountered an error: ... does not provide an export named 'init_logging'`, it's a stale browser cache, not a server problem — hard refresh (Cmd+Shift+R) or open in a private window.
  - Performance is genuinely limited by software rendering under amd64 emulation (no GPU passthrough) — expect roughly 4–5 FPS / ~0.5 real-time factor in the Gazebo GUI itself. The headless camera feed into Foxglove is unaffected by GUI performance.

### Why the robot doesn't move

The URDF currently has `<gazebo><static>true</static></gazebo>` set. Without a joint controller (`libgazebo_ros_control.so` was removed early on), an unconstrained 6-DOF arm free-falls and oscillates under gravity once collision meshes load correctly — `static` freezes physics so the camera shows a clean, stable scene. Swap this for a real `ros2_control` controller when wiring in actual joint actuation.

### Camera topics published

| Topic | Content |
|---|---|
| `/camera/color/image_raw` | RGB image, 640×480 |
| `/camera/color/camera_info` | Color camera intrinsics |
| `/camera/depth/image_raw` | Depth image |
| `/camera/depth/camera_info` | Depth camera intrinsics |
| `/camera/depth/points` | XYZRGB point cloud |

---

## macOS CLI (Optional)

Inspect topics from macOS using the Zenoh bridge:

```bash
# Terminal — start Zenoh bridge first
./scripts/start_bridge.sh

# Then use the ros2 wrapper
./scripts/ros2.sh topic list
./scripts/ros2.sh topic echo /joint_states
./scripts/ros2.sh node list
```

---

## Stop Everything

```bash
# Ctrl+C in each terminal to stop individual nodes
docker compose down
```

---

## Known Issues and Fixes

| Issue | Fix |
|---|---|
| Container bridge IP unreachable from macOS | Zenoh bridge over TCP port 7447 |
| Homebrew Python 3.14 breaks rclpy imports | `scripts/ros2.sh` unsets `PYTHONPATH`, uses conda Python directly |
| `ros2 topic list` hangs 60 s | `--no-daemon` injected by wrapper |
| URDF links appear disconnected in Foxglove | Set **Scene → Mesh "up" axis → Y-up** in the 3D panel |
| CycloneDDS interface mismatch on macOS | `CYCLONEDDS_URI` config; `ROS_LOCALHOST_ONLY=1` on bridge |
| Gazebo Classic apt deps broken on arm64 (`libignition-sensors6` version mismatch) | Build container as `linux/amd64` (emulated) |
| Robot meshes (visual + collision) fail to load in Gazebo (`Failed to find mesh file [model://...]`) | Add `<gazebo_ros gazebo_model_path="${prefix}/.."/>` export to each mesh-owning package's `package.xml`, and export `GAZEBO_MODEL_PATH`/`GAZEBO_RESOURCE_PATH` before starting `gzserver` |
| `gazebo_ros_camera.so` ignores `cameraName`/`imageTopicName` tags (ROS1-style) | Use `<ros><remapping>...</remapping></ros>` + `camera_name`/`frame_name` (ROS2-style) instead |
| Spawned robot flails/falls under gravity | No `ros2_control` controller is loaded; either add one, or set `<gazebo><static>true</static></gazebo>` to freeze physics |
| `spawn_entity.py` shows stale robot after editing the URDF | `robot_state_publisher` reads the file once at startup — restart it after `colcon build` before respawning |
| noVNC: `does not provide an export named 'init_logging'` | Stale browser cache — hard refresh or use a private window |
| `gzclient` hangs forever on **OrbStack** (stuck right after `GuiIface.cc` warnings, 0% CPU, no window ever appears) | Not fixable from this repo — incompatibility between OrbStack's amd64 emulation and Mesa's software OpenGL renderer, specific to the GUI's real-time render loop. Confirmed reproducible. Disabling OrbStack's Rosetta setting (`orbctl config set rosetta false`) makes it worse — `gzserver` itself then crashes (`LLVM ERROR: 64-bit code requested on a subtarget that doesn't support it!`). Keep `rosetta: true`. The headless camera pipeline (gzserver → Foxglove) is unaffected either way and fully works on OrbStack. If you need the interactive GUI, use Docker Desktop instead — it works there. |

---

## Roadmap

- [x] ROS2 Docker infrastructure (Zenoh bridge, Foxglove WebSocket)
- [x] xArm6 URDF + gripper meshes integrated as ROS2 packages
- [x] Robot model visualized and animated in Foxglove
- [x] Gripper open/close animation working
- [ ] Orbbec Astra camera integration (SDK + ROS2 driver)
- [ ] Point cloud processing (PCL — plane removal, clustering, 3D bounding box)
- [ ] Camera-to-robot TF2 calibration
- [ ] Conveyor state estimator (position tracking)
- [ ] Task scheduler with deadline-based queue
- [ ] MoveIt2 setup for xArm6 (IK, collision checking)
- [ ] Pick planner (antipodal grasp, yaw alignment)
- [ ] Container tracker + index controller
- [ ] Foxglove simulation (conveyor_sim, camera_sim, container_sim)
- [ ] Full end-to-end pick-and-place demo

---

## License

MIT
