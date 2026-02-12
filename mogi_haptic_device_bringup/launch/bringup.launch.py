import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_bringup = get_package_share_directory('mogi_haptic_device_bringup')

    arm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, 'launch', 'roarm.launch.py'),
        )
    )

    tool_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, 'launch', 'telemanipulator.launch.py'),
        )
    )

    launchDescriptionObject = LaunchDescription()

    launchDescriptionObject.add_action(arm_launch)
    launchDescriptionObject.add_action(tool_launch)

    return launchDescriptionObject