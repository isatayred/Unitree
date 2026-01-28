from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    g1_share = get_package_share_directory('g1_description')

    # Change this if your URDF filename is different
    urdf_path = os.path.join(g1_share, 'urdf', 'g1_23dof.urdf')

    robot_description = open(urdf_path).read()

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
    )

    return LaunchDescription([rsp, rviz])

