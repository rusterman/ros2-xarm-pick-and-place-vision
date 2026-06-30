# ros2-cheese-pick-place

A ROS2 robotic system for **detecting, picking, and placing cheese slices into a moving grid container** using computer vision and motion planning. Built with a fully containerized architecture that runs on **macOS (Apple Silicon / M chips)** out of the box.

> **Status:** Active development — infrastructure complete; the workspace has been reorganized into a layered, Clean-Architecture package structure (see [docs/PACKAGE_STRUCTURE.md](docs/PACKAGE_STRUCTURE.md)). The full cell — xArm6 + gripper, Orbbec Astra 2 camera, and a parametric conveyor belt — is composed by `cell_description` and visualized as a static scene in Foxglove via `estica_bringup`. The full Gazebo Classic camera-rendering pipeline is documented and was previously working; its launch package was removed during the reorg and is being re-integrated into the new layout. Perception and manipulation are next.

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

This is the target pipeline ([docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)). The sections below describe what is built **today**.

---

## Technology Stack

| Layer | Technology |
|---|---|
| Robot middleware | ROS2 Humble |
| Robot hardware | UFACTORY xArm6 (6-DOF collaborative arm) |
| Gripper | Mechanical parallel gripper with widened fingertips |
| Camera | Orbbec Astra 2 (structured light, RGB-D) |
| Visualization | Foxglove Studio (via WebSocket bridge) |
| Computer vision | OpenCV + PCL (geometric detection, no AI/ML) |
| Motion planning | MoveIt2 |
| Container runtime | Docker (linux/amd64 — emulated on Apple Silicon, required for Gazebo Classic's apt dependency chain) |
| Simulation | Gazebo Classic 11 (`gazebo_ros_pkgs`) — headless, Xvfb + Mesa software rendering |
| Remote GUI | noVNC (`x11vnc` + `websockify`) — view the real Gazebo GUI in a browser |
| macOS bridge | Zenoh (TCP bridge over Docker port-forward) |

**No AI/ML.** The real xArm6 hardware is available from day one. Object detection uses geometric PCL processing — sufficient for geometrically consistent cheese slices within a batch.

---

## Architecture

### How the workspace is organized

The repository follows a layered, Clean-Architecture package split — geometry, message contracts, decision logic, hardware, and bringup are kept in strictly separate packages so simulation and real hardware are swappable without touching the core logic. The full rationale (the Dependency Rule, the sim/real swap, why the conveyor is treated differently) is in **[docs/PACKAGE_STRUCTURE.md](docs/PACKAGE_STRUCTURE.md)**.

Only the `description/` and `bringup/` layers are scaffolded so far; the `interfaces/`, `functional/`, `hardware/`, `simulation/`, and `operational/` layers are designed but not yet built.

### System overview (full Gazebo pipeline — being re-integrated)

```
[ Apple Silicon Mac Host ]
       │
       ├──► Browser (localhost:6080) ──► [ noVNC / fluxbox Desktop ] ──► Gazebo GUI (gzclient, CPU-rendered)
       │
       └──► Foxglove Studio ◄── [ WebSocket :8765 (foxglove_bridge) ] ◄── ROS2 Topics (/tf, /camera/...)
                                                                              ▲
 ─────────────────────────────────────────────────────────────────────────────┼──────────────────────
[ OrbStack / Docker Container (Linux amd64, emulated) ]                       │
                                                                              │
  [ gzserver (Gazebo Classic) ] ──(built-in gazebo_ros plugins)───────────────┘
        │
        └─► (Renders Orbbec Astra camera viewpoint frame-by-frame, headless via Xvfb + Mesa)
```

Full breakdown of this diagram, the launch sequencing, and why each piece exists: [docs/GAZEBO_SIMULATION.md](docs/GAZEBO_SIMULATION.md). **Note:** the launch package that drove this pipeline (`cell_bringup`) was removed during the workspace reorg; the diagram and reasoning still hold, but the launch files need re-creating under the new `bringup/` layout.

### Why Zenoh, separately, for the macOS CLI

Docker/OrbStack runs containers inside a Linux VM. The container's bridge IP is unreachable from the host, so raw DDS does not work across that boundary. Zenoh bridges ROS2 topics over TCP — which Docker port-forwarding supports cleanly — purely so `ros2` CLI commands can run natively from macOS. It's unrelated to the Foxglove/noVNC paths above, which use a WebSocket and a VNC connection respectively.

---

## Repository Structure

```
.
├── Dockerfile                  # ros:humble + Gazebo Classic 11 + noVNC stack + zenoh-bridge
├── docker-compose.yml          # ports 7447 (zenoh), 8765 (foxglove), 6080 (noVNC)
├── src/
│   ├── description/                       # DATA ONLY — pure URDF/xacro + meshes, zero logic
│   │   ├── xarm_description/              # xArm6 + gripper URDF (xarm6_with_gripper.urdf.xacro)
│   │   ├── xarm_gripper/                  # gripper meshes only
│   │   ├── orbbec_description/            # Orbbec Astra 2 URDF (astra_2.urdf.xacro) + meshes
│   │   ├── conveyor_description/          # parametric conveyor belt (conveyor.urdf.xacro)
│   │   ├── container_description/         # (placeholder — not yet modeled)
│   │   ├── cheese_description/            # (placeholder — not yet modeled)
│   │   └── cell_description/              # composes all of the above into one scene
│   │       └── urdf/cell.urdf.xacro       #   — the ONLY package that positions things
│   └── bringup/
│       └── estica_bringup/               # launch files only
│           └── launch/cell_display.launch.py
├── xarm-source/                # Vendor-original UFACTORY xArm6 source (reference only)
├── config/
│   └── cyclonedds_macos.xml    # CycloneDDS loopback config
├── bin/                        # gitignored — populated by download_zenoh.sh
│   └── zenoh-bridge-ros2dds
├── scripts/
│   ├── download_zenoh.sh       # downloads macOS Zenoh bridge binary (run once)
│   ├── start_bridge.sh         # starts Docker router + macOS client bridge
│   └── ros2.sh                 # ros2 CLI wrapper (fixes Homebrew Python conflict)
└── docs/
    ├── ARCHITECTURE.md         # full system architecture (the target pipeline)
    ├── PACKAGE_STRUCTURE.md    # why the workspace is split this way (Clean Architecture)
    ├── DESIGN.md               # design notes
    ├── GAZEBO_SIMULATION.md    # Gazebo install/launch logic + diagrams (pipeline being re-integrated)
    └── ...
```

Every `description/` package is built with `ament_cmake` (a `CMakeLists.txt` that installs `urdf/` and `meshes/`), not `setup.py`. Per [docs/PACKAGE_STRUCTURE.md](docs/PACKAGE_STRUCTURE.md), `cell_description` is the single exception allowed to *position* things — every other description package stays pure geometry with its own frame at the origin.

### The conveyor model

`conveyor_description/urdf/conveyor.urdf.xacro` models the belt parametrically (length / width / thickness as xacro properties) as a thin box slab on a `conveyor_base_link` reference frame placed at the belt's top surface, so `cell_description` can mount it by belt-surface height directly. The belt slab is its own link joined by a **prismatic** `conveyor_belt_joint` (travel along **+Y**), ready for the [IFRA ConveyorBelt](https://github.com/IFRA-Cranfield/IFRA_ConveyorBelt) Gazebo plugin (Apache-2.0) — the `<gazebo>` plugin block is present but **dormant** until the Gazebo pipeline is re-integrated, since belt motion is a physics-sim effect and does not appear in the static Foxglove display.

---

## Prerequisites

- **Docker Desktop for Mac** or **OrbStack** (Apple Silicon)
- **Foxglove Studio** — [download](https://foxglove.dev/download)
- **Miniforge** with a `ros2` conda environment (RoboStack, ROS2 Humble) — for the optional macOS CLI only

---

## Quick Start

```bash
# 1. Clone
git clone https://github.com/rusterman/ros2-cheese-pick-place.git
cd ros2-cheese-pick-place

# 2. Download the macOS Zenoh bridge binary (run once, only needed for the macOS CLI)
./scripts/download_zenoh.sh

# 3. Build and start the container
docker compose build
docker compose up -d
```

---

## Run the Cell Display (current default)

Brings up the composed cell — xArm6 + gripper, Orbbec Astra 2, and the conveyor belt — as a static scene published over TF and streamed to Foxglove. No Gazebo, no physics: `robot_state_publisher` + `joint_state_publisher` (constant zero pose) + `foxglove_bridge`.

```bash
# Build the workspace (first time, or after adding/renaming packages or editing CMakeLists/package.xml)
docker exec ros2_dev bash -c "
  source /opt/ros/humble/setup.bash &&
  cd /ros2_ws &&
  colcon build --symlink-install"

# Launch the display
docker exec -it ros2_dev bash -c "
  source /opt/ros/humble/setup.bash &&
  source /ros2_ws/install/setup.bash &&
  ros2 launch estica_bringup cell_display.launch.py"
```

> `--symlink-install` makes the URDF/xacro/launch files live: after editing them you only need to **relaunch**, not rebuild. A full `colcon build` is only required when you add a new package, rename files, or change `package.xml`/`CMakeLists.txt`. If you ever hit `Failed to find ... package.sh` (stale colcon cache after a reorg), do `rm -rf /ros2_ws/{build,install,log}` and rebuild the whole workspace once.

### Viewing it

- **Foxglove Studio** → Open Connection → `ws://localhost:8765`
  - **3D panel** → add the robot model; you should see the arm, gripper, camera, and grey conveyor belt assembled into one scene.

---

## Gazebo Classic Simulation (real rendered camera + browser GUI — being re-integrated)

The full simulation runs the cell inside Gazebo Classic 11 with the Orbbec camera's actual rendered RGB-D output flowing into Foxglove, and the Gazebo GUI viewable in a browser tab via noVNC. This pipeline previously worked end-to-end; its launch package (`cell_bringup`) was removed during the workspace reorg and needs re-creating under `bringup/`. The container, Dockerfile, and reasoning are unchanged — see **[docs/GAZEBO_SIMULATION.md](docs/GAZEBO_SIMULATION.md)** for the architecture, launch sequencing, and the camera topics it publishes:

| Topic | Content |
|---|---|
| `/camera/color/image_raw` | RGB image, 640×480 |
| `/camera/color/camera_info` | Color camera intrinsics |
| `/camera/depth/image_raw` | Depth image |
| `/camera/depth/camera_info` | Depth camera intrinsics |
| `/camera/depth/points` | XYZRGB point cloud |

The container must be built for `linux/amd64` (already set in `docker-compose.yml`) — Gazebo's apt dependency chain is broken on arm64.

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
# Ctrl+C in the launch terminal to stop the nodes
docker compose down
```

---

## Known Issues and Fixes

| Issue | Fix |
|---|---|
| Container bridge IP unreachable from macOS | Zenoh bridge over TCP port 7447 |
| Homebrew Python 3.14 breaks rclpy imports | `scripts/ros2.sh` unsets `PYTHONPATH`, uses conda Python directly |
| `ros2 topic list` hangs 60 s | `--no-daemon` injected by wrapper |
| URDF links / meshes appear disconnected in Foxglove | Set **Scene → Mesh "up" axis → Y-up** in the 3D panel — this is a viewer setting, not a URDF bug |
| `$(find-pkg-share pkg)` fails inside a `.xacro` | That substitution only exists in launch files; inside xacro use `$(find pkg)` |
| `colcon build` fails: `Failed to find ... package.sh` | Stale install space after a reorg — build the whole workspace once (drop `--packages-select`), or `rm -rf /ros2_ws/{build,install,log}` and rebuild |
| CycloneDDS interface mismatch on macOS | `CYCLONEDDS_URI` config; `ROS_LOCALHOST_ONLY=1` on bridge |
| Gazebo Classic apt deps broken on arm64 (`libignition-sensors6` version mismatch) | Build container as `linux/amd64` (emulated) |
| Robot meshes fail to load in Gazebo (`Failed to find mesh file [model://...]`) | Add `<gazebo_ros gazebo_model_path="${prefix}/.."/>` export to each mesh-owning package's `package.xml`, and export `GAZEBO_MODEL_PATH`/`GAZEBO_RESOURCE_PATH` before starting `gzserver` |
| `gazebo_ros_camera.so` ignores ROS1-style tags | Use `<ros><remapping>...</remapping></ros>` + `camera_name`/`frame_name` (ROS2-style) instead |
| Spawned robot flails/falls under gravity | No `ros2_control` controller is loaded; either add one, or set `<gazebo><static>true</static></gazebo>` to freeze physics |
| noVNC: `does not provide an export named 'init_logging'` | Stale browser cache — hard refresh (Cmd+Shift+R) or use a private window |
| `gzclient` hangs forever on **OrbStack** (stuck after `GuiIface.cc` warnings, 0% CPU, no window) | OrbStack amd64-emulation × Mesa software-OpenGL incompatibility, specific to the GUI render loop. Keep `rosetta: true` (disabling it makes `gzserver` itself crash with `LLVM ERROR: 64-bit code requested...`). The headless camera pipeline is unaffected. Use Docker Desktop if you need the interactive GUI. |

---

## Roadmap

- [x] ROS2 Docker infrastructure (Zenoh bridge, Foxglove WebSocket)
- [x] xArm6 URDF + gripper meshes integrated as ROS2 packages
- [x] Robot model visualized and animated in Foxglove
- [x] Gripper open/close animation working
- [x] Workspace reorganized into layered package structure (`description/`, `bringup/`)
- [x] Orbbec Astra 2 camera modeled and placed in the cell
- [x] `cell_description` composes the full scene; static Foxglove display via `estica_bringup`
- [x] Conveyor belt modeled (`conveyor_description`) and placed in the cell
- [ ] Re-integrate the Gazebo Classic pipeline under `bringup/` (real rendered camera + noVNC GUI)
- [ ] Container + cheese descriptions (`container_description`, `cheese_description`)
- [ ] Point cloud processing (PCL — plane removal, clustering, 3D bounding box)
- [ ] Camera-to-robot TF2 calibration
- [ ] Conveyor state estimator (position tracking)
- [ ] Task scheduler with deadline-based queue
- [ ] MoveIt2 setup for xArm6 (IK, collision checking)
- [ ] Pick planner (antipodal grasp, yaw alignment)
- [ ] Container tracker + index controller
- [ ] Full end-to-end pick-and-place demo

---

## License

MIT
