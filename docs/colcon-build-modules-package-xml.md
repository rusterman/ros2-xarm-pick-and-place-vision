# ROS2 Package Structure

## src/ layout

`src/` is a container colcon scans to find packages. Each subfolder is an independent package.

```
src/
├── hello_ros2/    ← package 1: string pub/sub + RViz2 marker
└── sensor_sim/    ← package 2: temperature sensor + alerts
```

Packages don't know about each other and can be built, run, or removed independently.

---

## Double-folder convention

Every ROS2 Python package has a double-folder structure:

```
src/
└── hello_ros2/          ← ROS2 package root (colcon sees this)
    ├── package.xml
    ├── setup.py
    ├── setup.cfg
    ├── resource/
    │   └── hello_ros2   ← empty marker file required by ROS2 package index
    └── hello_ros2/      ← Python module (your actual code)
        ├── __init__.py
        ├── publisher.py
        ├── subscriber.py
        └── marker_publisher.py
```

| Folder | Role |
|--------|------|
| Outer `hello_ros2/` | ROS2 package — holds metadata, invisible to Python |
| Inner `hello_ros2/` | Python module — the importable code |

This mirrors standard Python packaging (same pattern as `pip` packages like `requests`).

---

## package.xml

Identity card of the package. colcon reads it before touching any Python.

```xml
<?xml version="1.0"?>
<?xml-model href="...schema...">         <!-- XML validation schema, no runtime effect -->
<package format="3">                     <!-- package format version, determines valid tags -->

  <name>hello_ros2</name>               <!-- must match inner folder name and setup.py -->
  <version>0.1.0</version>              <!-- used for dependency resolution -->
  <description>...</description>        <!-- shown by ros2 pkg list -->
  <maintainer email="...">...</maintainer>
  <license>Apache-2.0</license>

  <exec_depend>rclpy</exec_depend>              <!-- Python ROS2 client, required by every node -->
  <exec_depend>std_msgs</exec_depend>           <!-- String message type -->
  <exec_depend>visualization_msgs</exec_depend> <!-- Marker message type -->
  <exec_depend>geometry_msgs</exec_depend>      <!-- Pose/transform message types -->
  <exec_depend>tf2_ros</exec_depend>            <!-- Coordinate frame transforms -->

  <export>
    <build_type>ament_python</build_type>       <!-- pure Python build pipeline (vs ament_cmake for C++) -->
  </export>

</package>
```

colcon fails the build early if any `exec_depend` is missing — preventing runtime crashes.

---

## colcon build

colcon (collective construction) is the ROS2 build tool — equivalent to `make` or `npm install`.

`colcon build` scans `src/`, resolves dependencies in order, and produces:

```
ros2_ws/
├── src/      ← source code (what you edit)
├── build/    ← intermediate artifacts
├── install/  ← runnable output  ← this is what matters
└── log/      ← build logs
```

`install/` contains the entry-point scripts that make `ros2 run` work:

```bash
ros2 run hello_ros2 publisher
ros2 run sensor_sim temperature_publisher
```

In this project `colcon build` runs **inside Docker** — the container must be running first.

```bash
docker compose up -d
docker exec -it ros2_dev bash -c "source /opt/ros/humble/setup.bash && colcon build"
```

`build/` and `install/` live only inside the container and are discarded on `docker compose down`. This is intentional — a fresh build each time guarantees nothing is stale.
