import os
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    xarm_share = get_package_share_directory('xarm_description')
    xarm_gripper_share = get_package_share_directory('xarm_gripper')
    orbbec_share = get_package_share_directory('orbbec_description')

    urdf_file = os.path.join(xarm_share, 'urdf', 'xarm6_with_gripper.urdf')
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    world_file = os.path.join(xarm_share, 'worlds', 'camera_test.world')

    # model:// mesh URIs (xarm_description, xarm_gripper, orbbec_description) only
    # resolve if their package share dirs' parents are on these paths — without this,
    # every robot mesh (visual AND collision) silently fails to load in Gazebo.
    extra_model_path = os.pathsep.join([
        os.path.join(xarm_gripper_share, '..'),
        os.path.join(xarm_share, '..'),
        os.path.join(orbbec_share, '..'),
    ])

    return LaunchDescription([
        # Gazebo's OGRE renderer needs a real X11 display even fully headless.
        SetEnvironmentVariable('DISPLAY', ':1'),
        SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1'),
        # Seed with Gazebo's own defaults (shader lib, built-in models) so gzserver.launch.py's
        # GazeboRosPaths lookup — which only returns our 3 packages above — appends to these
        # instead of replacing them; without the defaults, OGRE can't find its shader lib.
        SetEnvironmentVariable('GAZEBO_RESOURCE_PATH', f'/usr/share/gazebo-11:{extra_model_path}'),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', f'/usr/share/gazebo-11/models:{extra_model_path}'),
        SetEnvironmentVariable('GAZEBO_PLUGIN_PATH', '/usr/lib/x86_64-linux-gnu/gazebo-11/plugins'),

        ExecuteProcess(
            cmd=['Xvfb', ':1', '-screen', '0', '1600x900x24'],
            output='screen',
        ),

        # Lightweight window manager — gives gzclient a real maximizable window
        # and a usable desktop (right-click for a launcher menu, e.g. a terminal)
        # instead of a bare floating window on a black background.
        TimerAction(period=1.0, actions=[
            ExecuteProcess(
                cmd=['fluxbox'],
                output='screen',
            ),
        ]),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),

        # Static hold pose — without a publisher on /joint_states, robot_state_publisher
        # has no data for the movable joints and their TF is left undefined.
        Node(
            package='xarm_description',
            executable='joint_state_pub',
            name='joint_state_pub',
            output='screen',
        ),

        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='screen',
            parameters=[{
                'port': 8765,
                'address': '0.0.0.0',
            }],
        ),

        TimerAction(period=3.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py')
                ),
                launch_arguments={'world': world_file, 'verbose': 'true'}.items(),
            ),
        ]),

        TimerAction(period=10.0, actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=['-topic', '/robot_description', '-entity', 'xarm6'],
                output='screen',
            ),
        ]),

        # noVNC stack — view the real Gazebo GUI in a browser at http://localhost:6080/vnc.html
        TimerAction(period=4.0, actions=[
            ExecuteProcess(
                # -ncache enables client-side pixel caching for smoother interaction
                cmd=['x11vnc', '-display', ':1', '-nopw', '-forever', '-shared', '-rfbport', '5900', '-ncache', '10'],
                output='screen',
            ),
        ]),

        TimerAction(period=5.0, actions=[
            ExecuteProcess(
                cmd=['websockify', '--web=/usr/share/novnc', '6080', 'localhost:5900'],
                output='screen',
            ),
        ]),

        TimerAction(period=6.0, actions=[
            ExecuteProcess(
                cmd=['gzclient', '--verbose'],
                output='screen',
            ),
        ]),

        # Maximize the gzclient window to fill the virtual display once it has appeared.
        TimerAction(period=9.0, actions=[
            ExecuteProcess(
                cmd=['xdotool', 'search', '--sync', '--name', 'Gazebo',
                     'windowsize', '1600', '868', 'windowmove', '0', '0'],
                output='screen',
            ),
        ]),
    ])
