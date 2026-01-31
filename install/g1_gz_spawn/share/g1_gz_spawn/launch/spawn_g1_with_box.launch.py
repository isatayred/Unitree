from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import random


def generate_launch_description():
    # choose a random box position inside the 4x4 room with a small margin
    margin = 0.6
    x = random.uniform(-2 + margin, 2 - margin)
    y = random.uniform(-2 + margin, 2 - margin)

    # ensure box isn't spawned too close to robot origin (0,0)
    if (x**2 + y**2) ** 0.5 < 0.8:
        x += 0.9

    this_pkg_share = get_package_share_directory('g1_gz_spawn')
    world_path = os.path.join(this_pkg_share, 'worlds', 'room_4x4.sdf')
    box_sdf_path = os.path.join(this_pkg_share, 'models', 'box.sdf')

    # Start Gazebo with the room world
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')
    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_path}'}.items()
    )

    # Load robot URDF for robot_state_publisher (with ros2_control)
    g1_description_share = get_package_share_directory('g1_description')
    urdf_path = os.path.join(g1_description_share, 'urdf', 'g1_23dof_control.urdf')
    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    robot_description = robot_description.replace(
        'package://g1_description/',
        f'file://{g1_description_share}/'
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    # Spawn G1 higher above ground for manual positioning
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-name', 'g1', '-topic', 'robot_description', '-x', '0', '-y', '0', '-z', '1.5'],
    )

    # Spawn box model at the chosen random position (box is 0.3m cube)
    spawn_box = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-file', box_sdf_path, '-name', 'box', '-x', str(x), '-y', str(y), '-z', '0.15'],
    )

    # Bridge Gazebo camera to ROS2
    camera_bridge = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            '/world/room_4x4_world/model/g1/link/head_link/sensor/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/world/room_4x4_world/model/g1/link/head_link/sensor/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'
        ],
        output='screen'
    )

    # Spawn joint state broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    # Spawn legs position controller (12 joints - both legs)
    legs_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['legs_position_controller'],
        output='screen'
    )

    return LaunchDescription([
        gz_launch,
        rsp,
        spawn_robot,
        spawn_box,
        camera_bridge,
        joint_state_broadcaster_spawner,
        legs_controller_spawner,
    ])
