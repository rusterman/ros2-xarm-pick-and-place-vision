# Gazebo Classic Simulation — Setup, Launch Logic, and Design Choices

Companion doc to the main [README.md](../README.md). The README has the commands to run; this doc explains *why* the stack looks the way it does, how the pieces talk to each other, and how the launch sequence is ordered.

---

## 1. Architecture

```
[ Apple Silicon Mac Host ]
       │
       ├──► Browser (localhost:6080) ──► [ noVNC / fluxbox Desktop ] ──► Gazebo GUI (gzclient, CPU-rendered)
       │
       └──► Foxglove Studio ◄── [ WebSocket :8765 (foxglove_bridge) ] ◄── ROS2 Topics (/camera/color/image_raw, /tf)
                                                                              ▲
 ─────────────────────────────────────────────────────────────────────────────┼──────────────────────
[ OrbStack Container (Linux amd64, emulated) ]                                │
                                                                              │
  [ gzserver (Gazebo Classic) ] ──(built-in gazebo_ros plugins)───────────────┘
        │
        └─► (Renders Orbbec Astra camera viewpoint frame-by-frame, headless via Xvfb + Mesa)
```

**Three independent communication paths exist, and none of them depend on each other:**

1. **ROS2 topics → Foxglove** — `gzserver` has `gazebo_ros` plugins compiled directly into it (`libgazebo_ros_init.so`, `libgazebo_ros_factory.so`, `libgazebo_ros_camera.so`). These publish straight onto the ROS2 DDS graph — there is no separate bridge process. `foxglove_bridge` (a normal ROS2 node) exposes everything on that graph over a WebSocket on port 8765, which Foxglove Studio connects to directly. **This is the only path that matters for the actual goal** (seeing real camera/sensor data).
2. **Browser → noVNC → Gazebo's own GUI** — `Xvfb` (virtual X11 display) → `x11vnc` (VNC server on that display) → `websockify` (wraps VNC in a WebSocket) → port 6080. This is pure remote-desktop pixel streaming. It carries zero ROS2 data — it's just a way to see Gazebo's native GUI window (`gzclient`) without installing a VNC client. Purely a convenience/debugging feature.
3. **Zenoh bridge** (not in the diagram, port 7447) — lets you run `ros2` CLI commands natively from macOS instead of `docker exec`. Unrelated to the other two paths.

---

## 2. Choices and why

### Gazebo Classic 11, not Ignition/`gz-sim`
Both were tried natively on macOS first and both failed for unrelated reasons:
- **Ignition/`gz-sim`** (conda-forge, native): physics and spawning worked, but the camera sensor crashed — `Ogre2RenderEngine::CreateRenderSystem()` segfault. Conda-forge's `ignition-rendering` can't create an OpenGL context on osx-arm64 with either OGRE backend.
- **Gazebo Classic** (conda-forge, native): crashed at world-load with `boost::wrapexcept<boost::lock_error>: mutex lock failed in pthread_mutex_lock` — a genuine ABI incompatibility, not a sandboxing issue.

Both pointed to the same conclusion: native macOS Gazebo (of either flavor) is not viable here. Docker was the only path left, and **Classic** was chosen over Ignition for that path because its ROS2 camera plugin (`libgazebo_ros_camera.so`) publishes directly to ROS2 topics with no separate bridge process to configure (Ignition needs `ros_gz_bridge` as an extra moving part).

### `linux/amd64`, emulated — not native `arm64`
Discovered while still evaluating Ignition: `libignition-gazebo6` requires `libignition-sensors6 >= 6.8.1`, but only `6.8.0` is published for `arm64` — an unfixable apt packaging gap. Once Docker was already the chosen path, `amd64` (via Rosetta/QEMU emulation) was kept for the Classic build too, since its apt repo has a more complete and consistent package set across `gazebo_ros_pkgs` and its dependency chain.

### Headless still needs `Xvfb`
Gazebo's OGRE renderer needs a real X11/OpenGL context to render *anything*, including the camera sensor's offscreen render — there is no true "no X server" mode. `Xvfb` provides that context with nothing actually displayed anywhere.

### `LIBGL_ALWAYS_SOFTWARE=1` (Mesa `llvmpipe`)
No GPU passthrough exists in Docker. This forces OGRE onto Mesa's software rasterizer instead of trying (and failing) to find a GPU.

### OrbStack over Docker Desktop — with one carve-out
OrbStack uses a measurably lighter VM and has much lower idle CPU/memory than Docker Desktop — the whole point of switching. Verified during this work:
- The **headless pipeline** (`gzserver` → camera → Foxglove) is byte-identical on both. No downside here.
- The **`gzclient` GUI** hangs permanently on OrbStack with its default `rosetta: true` (stuck at ~0% CPU right after Qt init, no window ever created). Disabling Rosetta (`orbctl config set rosetta false`) makes it *worse* — `gzserver` itself then crashes (`LLVM ERROR: 64-bit code requested on a subtarget that doesn't support it!`). This is a real incompatibility between OrbStack's emulation and Mesa's JIT-compiled software rendering for the GUI's real-time render loop specifically, not something fixable in this repo.
- **Decision**: keep OrbStack with `rosetta: true` as the daily driver. Use Docker Desktop only in the rare case the interactive `gzclient` window is actually needed.

### noVNC stack: `x11vnc` + `websockify` + `fluxbox`
- `x11vnc` + `websockify` is the standard lightweight way to expose an X11 display as a browser-viewable VNC session — no heavier alternative (e.g. a full VNC desktop environment) was needed.
- `fluxbox` was added after the GUI initially appeared as a bare undecorated window floating on a black background. It provides a real desktop (taskbar, maximizable/movable windows) at minimal resource cost — chosen over heavier environments (XFCE, LXDE) specifically to stay light.
- `gzclient` is auto-maximized via `xdotool` after launch, and the noVNC page is opened with `?resize=scale` so the browser scales the view to fill the window client-side — free (no extra server-side rendering), as opposed to asking the VNC server to re-render at a different resolution.

### `<gazebo><static>true</static></gazebo>` on the robot
No `ros2_control` controller is loaded (deliberately — building pick/place control logic is out of scope for this repo, see §4). Once collision meshes were loading correctly, an unconstrained 6-DOF arm under real gravity free-falls and oscillates unpredictably. `static` freezes physics for the robot so the camera shows a clean, stable scene. This will need to be swapped for a real controller once arm actuation is actually wired in.

### `joint_state_pub` node
`robot_state_publisher` only knows the pose of *fixed* joints from the URDF alone — it needs live `/joint_states` messages to compute TF for movable joints. With nothing publishing that topic, TF for the arm and gripper joints is simply undefined (this caused real, reproducible visual glitching in Foxglove before this was added). `joint_state_pub` publishes a constant, sane pose at 10 Hz.

### ROS2-style `<ros><remapping>` instead of legacy `cameraName`/`imageTopicName`
The ROS1-era tags (`cameraName`, `imageTopicName`, `frameName`, etc.) are silently ignored by the ROS2 port of `libgazebo_ros_camera.so` — they don't error, they just do nothing, which makes this bug easy to miss. The ROS2 port instead reads `<ros><namespace>`/`<remapping>` plus snake_case `camera_name`/`frame_name`.

### `<gazebo_ros gazebo_model_path="${prefix}/.."/>` export in `package.xml`
`model://<package>/meshes/...` URIs only resolve if `GAZEBO_MODEL_PATH` contains the parent of that package's share directory. Gazebo's launch tooling (`GazeboRosPaths.get_paths()`) only populates `GAZEBO_MODEL_PATH` from packages that explicitly declare this export — without it, **every** robot mesh (visual *and* collision) silently fails to load, with no fatal error, just an invisible/non-collidable robot.

### Foxglove Studio, not RViz
No X11/display server needed on the client side at all — connects over a plain WebSocket, so it runs natively and identically from the macOS host without any X11 forwarding or remote-desktop layer (unlike the `gzclient` GUI, which needs one).

### Zenoh bridge, separate from the Foxglove path
For native `ros2` CLI usage from macOS. Docker's container network isn't reachable for raw DDS multicast discovery from the host, but Zenoh bridges ROS2 topics over a plain TCP port, which Docker/OrbStack port-forwarding supports cleanly.

---

## 3. Installation

```bash
# Prerequisites: OrbStack (or Docker Desktop) + Foxglove Studio on the host

git clone <repo>
cd estica-solutions

docker compose build      # builds the linux/amd64 image (Gazebo Classic, ROS2 Humble, noVNC stack)
docker compose up -d      # starts the container

docker exec ros2_dev bash -c "
  source /opt/ros/humble/setup.bash &&
  cd /ros2_ws &&
  colcon build --symlink-install --packages-select xarm_description xarm_gripper orbbec_description"
```

`--symlink-install` matters for iteration speed: Python source files are symlinked into the install space, so editing existing node logic needs **no rebuild at all** — just stop and re-run. A rebuild is only needed when adding a brand-new file, a new `console_scripts` entry, or a new `package.xml` dependency.

---

## 4. Launch logic

```bash
docker exec -it ros2_dev bash -c "
  source /opt/ros/humble/setup.bash &&
  source /ros2_ws/install/setup.bash &&
  ros2 launch xarm_description gazebo_camera_test.launch.py"
```

One command brings up the entire stack. Internally, [`gazebo_camera_test.launch.py`](../src/xarm_description/launch/gazebo_camera_test.launch.py) sequences things like this:

```
t=0s   SetEnvironmentVariable: DISPLAY, LIBGL_ALWAYS_SOFTWARE,
       GAZEBO_RESOURCE_PATH / GAZEBO_MODEL_PATH / GAZEBO_PLUGIN_PATH
       (seeded with Gazebo's own defaults first, so gzserver.launch.py's
       package-path lookup appends to them instead of replacing them)
       │
       ├─► Xvfb :1                       (virtual display — everything else needs it)
       ├─► robot_state_publisher         (no Gazebo dependency, starts immediately)
       ├─► joint_state_pub               (no Gazebo dependency, starts immediately)
       └─► foxglove_bridge               (no Gazebo dependency, starts immediately)

t=1s   ├─► fluxbox                       (needs Xvfb up; small delay avoids a connection race)

t=3s   └─► gzserver (via gzserver.launch.py, includes camera_test.world)
                                          (needs Xvfb; 3s gives it time to be ready)

t=4s   ├─► x11vnc -display :1            (needs Xvfb)
t=5s   ├─► websockify :6080 → :5900      (needs x11vnc's port open)
t=6s   └─► gzclient                      (needs gzserver fully up)

t=9s   └─► xdotool: maximize the Gazebo window
                                          (needs gzclient's window to actually exist)

t=10s  └─► spawn_entity.py -topic /robot_description -entity xarm6
                                          (needs gzserver's /spawn_entity service ready —
                                           longest delay, since this is the step most
                                           likely to fail silently if gzserver isn't
                                           fully initialized yet)
```

The delays are empirically-tuned `TimerAction` offsets, not topic/service `wait_for`-based — simple but means the timings could need adjusting if the host is under heavy load (e.g. slow disk/cold cache making `gzserver` take longer than 10s to expose its spawn service).

---

## 5. Roadmap

| Status | Item | Lives in |
|---|---|---|
| ✅ Done | xArm6 + gripper + Orbbec camera, real rendering verified into Foxglove | Gazebo |
| ✅ Done | Single-command launch for the whole pipeline | — |
| ✅ Done | noVNC bonus GUI (Docker Desktop only — see §2) | Gazebo + noVNC stack |
| 🔜 Next | Conveyor belt — physics-driven (friction-based surface velocity, not scripted poses) | Gazebo (world model + plugin) |
| 🔜 Next | Cheese piece spawner — continuous node, calls `/spawn_entity` on a timer | ROS2 node (ament_python) |
| 🔜 Next | Grasp *mechanism* — attach/detach service that creates/removes a physics joint | Gazebo plugin |
| 🚫 Not built here | Grasp *decision logic*, detection algorithm, IK/motion planning | The user's own separate control program — calls the services above, isn't part of this repo |

The boundary on that last row is deliberate, not an oversight: this repo's job is the simulated cell and clean ROS2 interfaces to it (camera topics, spawn/attach/detach services). The actual cheese-detection and pick-place control logic is a separate program that consumes those interfaces.
