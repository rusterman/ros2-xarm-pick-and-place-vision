import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import (
    Command, EnvironmentVariable, FindExecutable, LaunchConfiguration,
    PathJoinSubstitution, TextSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

_XACRO_NS = "{http://ros.org/wiki/xacro}"


def _xacro_defaults(package, xacro_relpath, arg_names):
    """Read <xacro:arg name="..." default="..."/> values straight out of a
    xacro file's own declarations, instead of re-typing a second hardcoded
    copy here that can drift out of sync. A .xacro file is well-formed XML
    even before macro expansion — xacro:arg is just a namespaced element —
    so this parses it with ElementTree rather than regex-scraping the source
    text, which would be sensitive to attribute order/whitespace that XML
    itself doesn't care about. Needs a real filesystem path right now
    (launch description construction time), not a deferred Substitution, so
    this uses get_package_share_directory rather than FindPackageShare."""
    xacro_path = os.path.join(get_package_share_directory(package), xacro_relpath)
    root = ET.parse(xacro_path).getroot()
    defaults = {el.get("name"): el.get("default") for el in root.iter(f"{_XACRO_NS}arg")}
    missing = [name for name in arg_names if name not in defaults]
    if missing:
        raise RuntimeError(f"xacro:arg {missing} not found in {xacro_path}")
    return {name: float(defaults[name]) for name in arg_names}


def generate_launch_description():
    use_gazebo = LaunchConfiguration("use_gazebo")

    # --- cell layout ---
    # conveyor_mount_* is a placement decision — where *this* launch puts the
    # conveyor in the world — which is cell_description's job to own (it's
    # the one package allowed to position things), so these stay
    # Python-authoritative here and get pushed down as xacro args.
    conveyor_mount_x = 0.40
    conveyor_mount_y = -0.8
    conveyor_mount_z = 0.01

    # belt_thickness and the cheese batch geometry/color, by contrast, are
    # intrinsic object properties that already have a canonical home in
    # their own description package's xacro defaults — read them back
    # instead of redefining them here. Only meaningful when use_gazebo:=true
    # (cheese spawning), but cheap enough to always compute.
    belt_thickness = _xacro_defaults(
        "conveyor_description", "urdf/conveyor.urdf.xacro", ["belt_thickness"]
    )["belt_thickness"]

    cheese_defaults = _xacro_defaults(
        "cheese_description", "urdf/cheese.urdf.xacro",
        ["length", "width", "height", "color_r", "color_g", "color_b", "color_a"],
    )
    cheese_length = cheese_defaults["length"]
    cheese_width = cheese_defaults["width"]
    cheese_height = cheese_defaults["height"]
    cheese_color_r = cheese_defaults["color_r"]
    cheese_color_g = cheese_defaults["color_g"]
    cheese_color_b = cheese_defaults["color_b"]
    cheese_color_a = cheese_defaults["color_a"]

    spawn_x_center = conveyor_mount_x
    spawn_z = conveyor_mount_z + belt_thickness / 2.0 + cheese_height / 2.0

    cell_xacro = PathJoinSubstitution(
        [FindPackageShare("cell_description"), "urdf", "cell.urdf.xacro"]
    )
    cell_xacro_args = TextSubstitution(
        text=(
            f" conveyor_mount_x:={conveyor_mount_x} conveyor_mount_y:={conveyor_mount_y}"
            f" conveyor_mount_z:={conveyor_mount_z}"
        )
    )
    robot_description = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", cell_xacro, cell_xacro_args]),
        value_type=str,
    )

    # No overrides passed — cheese.urdf.xacro's own defaults are exactly
    # what was just read above, so the rendered URDF and the cheese_spawner
    # node parameters below are guaranteed to describe the same cheese.
    cheese_xacro = PathJoinSubstitution(
        [FindPackageShare("cheese_description"), "urdf", "cheese.urdf.xacro"]
    )
    cheese_urdf = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", cheese_xacro]),
        value_type=str,
    )

    world = PathJoinSubstitution([FindPackageShare("estica_gazebo"), "worlds", "cell.world"])

    # Only packages that own actual meshes need to be on GAZEBO_MODEL_PATH —
    # cheese_description/conveyor_description/cell_description are box-primitive-only.
    gazebo_model_path = [
        PathJoinSubstitution([FindPackageShare("orbbec_description"), ".."]),
        ":",
        PathJoinSubstitution([FindPackageShare("xarm_description"), ".."]),
        ":",
        PathJoinSubstitution([FindPackageShare("xarm_gripper"), ".."]),
        ":",
        EnvironmentVariable("GAZEBO_MODEL_PATH", default_value=""),
    ]

    # --- nodes shared by both backends (static display and Gazebo) ---
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    # No controller is wired in — this publishes a constant zero pose on
    # /joint_states so robot_state_publisher has TF data for the arm's
    # movable joints (otherwise they're undefined).
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
    )

    foxglove_bridge = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        output="screen",
        parameters=[{"port": 8765}],
    )

    # --- Gazebo-only actions, gated on use_gazebo:=true ---
    gzserver = ExecuteProcess(
        cmd=[
            "gzserver",
            "--verbose",
            "-s",
            "libgazebo_ros_init.so",
            "-s",
            "libgazebo_ros_factory.so",
            world,
        ],
        additional_env={"GAZEBO_MODEL_PATH": gazebo_model_path},
        output="screen",
        condition=IfCondition(use_gazebo),
    )

    spawn_cell = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=["-topic", "robot_description", "-entity", "estica_cell", "-timeout", "120"],
        condition=IfCondition(use_gazebo),
    )

    cheese_spawner = Node(
        package="estica_gazebo",
        executable="cheese_spawner",
        output="screen",
        parameters=[{
            "cheese_urdf": cheese_urdf,
            "belt_speed": 0.10,
            "spawn_interval_min": 4.0,
            "spawn_interval_max": 6.0,
            "spawn_x_center": spawn_x_center,
            "spawn_x_jitter": 0.10,
            "spawn_y": -1.5,
            "spawn_z": spawn_z,
            "despawn_y": 2.0,
            "cheese_length": cheese_length,
            "cheese_width": cheese_width,
            "cheese_height": cheese_height,
            "cheese_color_r": cheese_color_r,
            "cheese_color_g": cheese_color_g,
            "cheese_color_b": cheese_color_b,
            "cheese_color_a": cheese_color_a,
        }],
        condition=IfCondition(use_gazebo),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_gazebo",
            default_value="false",
            description=(
                "If true, also start gzserver and spawn the cell + cheese "
                "into Gazebo Classic physics sim. If false (default), only "
                "the static TF display (robot_state_publisher + "
                "joint_state_publisher + foxglove_bridge) is brought up."
            ),
        ),
        robot_state_publisher,
        joint_state_publisher,
        foxglove_bridge,
        gzserver,
        RegisterEventHandler(
            OnProcessStart(
                target_action=gzserver,
                on_start=[spawn_cell, cheese_spawner],
            ),
            condition=IfCondition(use_gazebo),
        ),
    ])
