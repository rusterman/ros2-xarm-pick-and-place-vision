from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    cell_xacro = PathJoinSubstitution(
        [FindPackageShare("cell_description"), "urdf", "cell.urdf.xacro"]
    )

    robot_description = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", cell_xacro]),
        value_type=str,
    )

    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description}],
            ),
            # No controller is wired in — this publishes a constant zero
            # pose on /joint_states so robot_state_publisher has TF data
            # for the arm's movable joints (otherwise they're undefined).
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="joint_state_publisher",
                output="screen",
            ),
            Node(
                package="foxglove_bridge",
                executable="foxglove_bridge",
                name="foxglove_bridge",
                output="screen",
                parameters=[{"port": 8765}],
            ),
        ]
    )
