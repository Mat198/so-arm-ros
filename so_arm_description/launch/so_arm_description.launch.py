import os
import xacro

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    description_pkg_path = get_package_share_directory("so_arm_description")
    robot_description_file = xacro.process_file(
        os.path.join(description_pkg_path,"urdf", "so_arm_101.urdf.xacro"
    ))

    launch_description = LaunchDescription()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            "robot_description": robot_description_file.toxml(),
            "publish_frequency": 50.0,
            "ignore_timestamp": False,
        }]
    )
    launch_description.add_action(robot_state_publisher_node)

    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )
    launch_description.add_action(joint_state_publisher_node)

    rviz_config_file = (os.path.join(description_pkg_path, "rviz", "so_arm.rviz"))
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[{
            "robot_description": robot_description_file.toxml(),
        }],
    )
    launch_description.add_action(rviz_node)

    return launch_description
