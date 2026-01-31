#!/usr/bin/env python3
"""Simple diagnostics for Gazebo / ros_gz_sim display problems.

Run after sourcing your ROS2 and workspace setup scripts. It checks:
- DISPLAY environment variable
- presence of `ros_gz_sim` package
- presence of `gz` or `ign` simulator binary in PATH
"""
import os
import shutil
import sys
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError


def main():
    print('--- Simulation environment diagnostics ---')

    disp = os.environ.get('DISPLAY')
    print('DISPLAY:', disp)

    # Check for simulator binaries
    gz = shutil.which('gz')
    ign = shutil.which('ign')
    print('gz binary:', gz)
    print('ign binary:', ign)

    # Check ros_gz_sim package
    try:
        p = get_package_share_directory('ros_gz_sim')
        print('ros_gz_sim share:', p)
    except PackageNotFoundError:
        print('ros_gz_sim: NOT FOUND (did you install the package?)')

    # Check our world file exists
    try:
        share = get_package_share_directory('g1_gz_spawn')
        world = os.path.join(share, 'worlds', 'room_4x4.sdf')
        print('room_4x4.sdf exists:', os.path.exists(world), world)
    except PackageNotFoundError:
        print('g1_gz_spawn package not found')

    print('\nIf DISPLAY is empty or you are on a headless server, run with X forwarding, use VNC, or run in headless mode.')
    print('To continue troubleshooting, run the launch and paste the terminal output here.')


if __name__ == '__main__':
    main()
