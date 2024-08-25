from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to the ldlidar_with_mgr.launch.py
    ldlidar_launch_path = os.path.join(
        get_package_share_directory('ldlidar_node'), 
        'launch', 
        'ldlidar_with_mgr.launch.py'
    )

    return LaunchDescription([
        # Launch the serial communicator node
        ExecuteProcess(
            cmd=['ros2', 'run', 'asa_control', 'serial_communicator'],
            output='screen'
        ),
        # Include the ldlidar_with_mgr.launch.py launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ldlidar_launch_path)
        ),
        # Launch minicom
        ExecuteProcess(
            cmd=['gnome-terminal', '--', 'minicom', '-b', '115200', '-o', '-D', '/dev/ttyACM0'],
            output='screen'
        )
    ])

