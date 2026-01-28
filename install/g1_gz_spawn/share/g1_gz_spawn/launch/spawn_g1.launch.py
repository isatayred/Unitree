from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # -----------------------------
    # Start Gazebo with custom room
    # -----------------------------
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')
    this_pkg_share = get_package_share_directory('g1_gz_spawn')

    world_path = os.path.join(
        this_pkg_share,
        'worlds',
        'room_4x4.sdf'
    )

    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_path}'}.items()
    )

    # -----------------------------
    # Load URDF CORRECTLY (ROS-safe)
    # -----------------------------
    g1_description_share = get_package_share_directory('g1_description')
    urdf_path = os.path.join(
        g1_description_share,
        'urdf',
        'g1_23dof.urdf'
    )

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    # Convert ROS package URIs to absolute file URIs for Gazebo Sim
    robot_description = robot_description.replace(
        'package://g1_description/',
        f'file://{g1_description_share}/'
    )

# (optional) if any model:// sneaks in from other files
    robot_description = robot_description.replace(
        'model://g1_description/',
        f'file://{g1_description_share}/'
    )

    # -----------------------------
    # Robot State Publisher
    # -----------------------------
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    # -----------------------------
    # Spawn robot into Gazebo
    # -----------------------------
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'g1',
            '-topic', 'robot_description',
            '-x', '0', '-y', '0', '-z', '1.0'
        ],
    )

    return LaunchDescription([
        gz_launch,
        rsp,
        spawn
    ])
