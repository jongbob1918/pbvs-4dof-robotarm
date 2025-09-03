import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    # Get the package share directory
    arm_description_pkg_share = get_package_share_directory('arm_description')

    # Define the path to the rviz config file
    default_rviz_config_path = os.path.join(arm_description_pkg_share, 'rviz/urdf_config.rviz')

    # Define the path to the urdf file
    urdf_file_name = 'arm.urdf.xacro'
    urdf_path = os.path.join(arm_description_pkg_share, 'urdf', urdf_file_name)

    # Get the robot description from the xacro file
    robot_description_config = xacro.process_file(urdf_path)
    robot_desc = robot_description_config.toxml()

    # Declare the launch arguments
    declare_rviz_config_arg = DeclareLaunchArgument(
        name='rvizconfig',
        default_value=default_rviz_config_path,
        description='Absolute path to rviz config file')

    # Create the robot_state_publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}])

    # Create the joint_state_publisher_gui node
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen')

    # Create the rviz2 node
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')])

    # Create the launch description
    ld = LaunchDescription([
        declare_rviz_config_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz2_node
    ])

    return ld