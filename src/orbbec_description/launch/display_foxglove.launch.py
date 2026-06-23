import os
import sys
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('orbbec_description')
    urdf_dir = os.path.join(pkg_share, 'urdf')

    available_models = [f for f in os.listdir(urdf_dir) if f.endswith('.urdf.xacro') and not f.startswith('test_')]

    model_arg = DeclareLaunchArgument(
        'model',
        default_value='astra_2.urdf.xacro',
        description='URDF xacro file to load. Available: ' + ', '.join(available_models),
    )

    model_name = LaunchConfiguration('model')

    def robot_description(context):
        model_file = context.launch_configurations['model']
        xacro_path = os.path.join(urdf_dir, model_file)
        doc = xacro.process_file(xacro_path, mappings={'use_nominal_extrinsics': 'true', 'add_plug': 'true'})
        robot_desc = doc.toprettyxml(indent='  ')
        return [
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{'robot_description': robot_desc}],
            ),
            Node(
                package='joint_state_publisher',
                executable='joint_state_publisher',
                name='joint_state_publisher',
                output='screen',
            ),
        ]

    from launch.actions import OpaqueFunction
    return LaunchDescription([
        model_arg,
        OpaqueFunction(function=robot_description),
    ])
