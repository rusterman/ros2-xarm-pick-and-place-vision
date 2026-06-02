# ROS2 Package Files: setup.py, setup.cfg, resource/

All three are required. Removing any one of them breaks the build or `ros2 run`.

---

## setup.py

Called by `colcon build` to install the package. Key parts:

```python
package_name = 'hello_ros2'  # reused below, must match package.xml

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),  # auto-discovers inner hello_ros2/ module

    data_files=[
        # registers package in ROS2's index (ros2 pkg list / ros2 run)
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # makes package.xml available at runtime for ros2 pkg commands
        ('share/' + package_name, ['package.xml']),
    ],

    entry_points={
        'console_scripts': [
            # creates runnable commands: ros2 run hello_ros2 publisher
            'publisher = hello_ros2.publisher:main',
            'subscriber = hello_ros2.subscriber:main',
            'marker_publisher = hello_ros2.marker_publisher:main',
        ],
    },
)
```

The `entry_points` block is the most critical part — it's what makes `ros2 run` work. Each line maps a command name to a Python function.

---

## setup.cfg

Tells setuptools where to put the generated executables inside `install/`:

```ini
[develop]
script_dir=$base/lib/hello_ros2

[install]
install_scripts=$base/lib/hello_ros2
```

ROS2 expects executables at `lib/<package_name>/`. Without this file, setuptools places them in the wrong path and `ros2 run` can't find them.

---

## resource/ folder

Contains a single empty file named `hello_ros2`. The content is irrelevant — its existence is the signal.

`setup.py` copies it to `share/ament_index/resource_index/packages/` during build. ROS2 scans that directory to discover installed packages. Without it, `ros2 pkg list` won't show the package and `ros2 run` won't find it.

---

## What breaks without each

| File | Consequence of removing |
|------|------------------------|
| `setup.py` | `colcon build` fails — nothing installs |
| `setup.cfg` | executables land in the wrong path, `ros2 run` fails |
| `resource/hello_ros2` | ROS2 doesn't know the package exists |
