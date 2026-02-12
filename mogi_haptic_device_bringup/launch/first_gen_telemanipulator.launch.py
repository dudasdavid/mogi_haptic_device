import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

  use_sim_time = LaunchConfiguration('use_sim_time')

  declare_use_sim_time_cmd = DeclareLaunchArgument(
    name='use_sim_time',
    default_value='False',
    description='Use simulation (Gazebo) clock if true')

  start_telemanipulator_driver = Node(
    package='mogi_haptic_device_py',
    executable='first_gen_driver',
    name='telemanipulator_driver',
    output='screen',
    parameters=[{'use_sim_time': use_sim_time}])
   
  # Create the launch description and populate
  ld = LaunchDescription()
 
  # Add any actions
  ld.add_action(declare_use_sim_time_cmd)
  ld.add_action(start_telemanipulator_driver)
 
  return ld