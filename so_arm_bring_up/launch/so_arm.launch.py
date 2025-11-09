import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node, SetParameter
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
      
        return None

def generate_launch_description():

    ############################## Launch parameters ###############################

    mock_controller_arg = DeclareLaunchArgument(
        'mock_controller', default_value="false", choices=["false", "true"],
        description="Enable mock controller when true. When false expects communication with " \
        "real hardware."
    )

    ########################## Loads the robot description #########################
    
    # Loading according to: 
    #   https://moveit.picknik.ai/main/doc/examples/urdf_srdf/urdf_srdf_tutorial.html

    config_pkg_path = FindPackageShare("so_arm_config")
    urdf_path = PathJoinSubstitution([config_pkg_path, "config", "so_arm_101.urdf.xacro"])

    robot_description_str = Command([# Remember to manually add space between commands
        "xacro ", urdf_path,
        " mock_controller:=", LaunchConfiguration("mock_controller"),
    ])
    robot_description = {'robot_description': ParameterValue(robot_description_str, value_type=str)}

    robot_description_semantic_str = load_file("so_arm_config", "config/so_arm_101.srdf")
    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(robot_description_semantic_str, value_type=str)
    }

    robot_description_kinematics = PathJoinSubstitution(
        [config_pkg_path, "config", "kinematics.yaml"])
    
    ######################## Loads the robot configuration #########################

    ompl_planning= PathJoinSubstitution([config_pkg_path, "config", "ompl_planning.yaml"])
   
    pilz_planning =  PathJoinSubstitution(
        [config_pkg_path, "config", "pilz_industrial_motion_planner_planning.yaml"])

    planning_pipelines = PathJoinSubstitution(
        [config_pkg_path, "config", "planning_pipelines.yaml"])
    
    joint_limits = PathJoinSubstitution([config_pkg_path, "config", "joint_limits.yaml"])

    cartesian_limits = PathJoinSubstitution(
        [config_pkg_path, "config", "pilz_cartesian_limits.yaml"])
    
    moveit_sensors = PathJoinSubstitution([config_pkg_path, "config", "sensors.yaml"])
    
        
    moveit_controllers = PathJoinSubstitution(
        [config_pkg_path, "config", "moveit_controllers.yaml"])

    trajectory_execution = PathJoinSubstitution(
        [config_pkg_path, "config", "trajectory_execution.yaml"])
    
    planning_scene_monitor = PathJoinSubstitution(
        [config_pkg_path, "config", "planning_scene_monitor.yaml"])

    ############################### Move Group node ################################

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description_semantic,
            robot_description_kinematics,
            cartesian_limits,
            joint_limits,

            ompl_planning,
            pilz_planning,
            planning_pipelines,
            
            moveit_controllers,
            trajectory_execution,
            moveit_sensors,
            planning_scene_monitor,
        ],
    )

    ############################### SO-ARM Manager ################################

    # Enables motors and check for unsafe conditions (high voltage, high temperature, ...)

    so_arm_manager_node = Node(
        package='so_arm_manager',
        executable='so_arm_manager',
        output='screen'
    )

    ############################## ROS 2 Controllers ###############################
    
    so_arm_controller = PathJoinSubstitution([config_pkg_path, "config", "ros2_controllers.yaml"])
    
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        # Don't add name. It will cause a duplicated node being created when spawn a controller
        parameters=[so_arm_controller],
        # It doesn't make much sense for me why they expect the robot description on topic
        # ~/robot_description and not /namespace/robot_description. They want a separeted robot
        # description for this node?
        remappings=[("/controller_manager/robot_description", "/robot_description")],
        output="screen",
    )

    so_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="so_arm_controller_spawner",
        arguments=["so_arm_controller"],
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="gripper_controller_spawner",
        arguments=["gripper_controller"],
    )
    
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="joint_state_broadcaster_spawner",
        arguments=["joint_state_broadcaster"],
    )

    joint_sensor_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="joint_sensor_broadcaster_spawner",
        arguments=["joint_sensor_broadcaster"],
    )

    motor_torque_enable_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="motor_torque_enable_controller_spawner",
        arguments=["motor_torque_enable_controller"],
    )

    ############################ Robot State Publisher #############################

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description,
            {"publish_frequency": 50.0, "ignore_timestamp": False,}
        ]
    )

    ##################################### RVIZ #####################################

    # TODO: Add parameter to disable RVIZ

    bring_up_pkg_path = FindPackageShare("so_arm_config")
    rviz_config_file = PathJoinSubstitution([bring_up_pkg_path, "rviz", "moveit.rviz"])
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
    )

    ############################## Launch Description ##############################

    return LaunchDescription([
        # Adds configurations to launch:

        # Adds arguments to launch:
        mock_controller_arg,

        # Adds nodes to launch:
        move_group_node,
        so_arm_manager_node,
        robot_state_publisher_node,
        rviz_node,
        controller_manager,
        so_arm_controller_spawner,
        gripper_controller_spawner,
        joint_state_broadcaster_spawner,
        joint_sensor_broadcaster_spawner,
        motor_torque_enable_controller_spawner,
    ])
