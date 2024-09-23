from moveit_configs_utils import MoveItConfigsBuilder
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.conditions import IfCondition
from ur_moveit_config.launch_common import load_yaml
from launch_ros.parameter_descriptions import ParameterValue

import os

def launch_setup(context, *args, **kwargs):
    # Extract launch configurations
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_rviz = LaunchConfiguration("launch_rviz")
    moveit_config_package = LaunchConfiguration("moveit_config_package")
    moveit_joint_limits_file = LaunchConfiguration("moveit_joint_limits_file")
    moveit_config_file = LaunchConfiguration("moveit_config_file")
    com_port = LaunchConfiguration("com_port")
    headless_mode = LaunchConfiguration("headless_mode")

    # Load the robot configuration 
    moveit_config = (
        MoveItConfigsBuilder("ur5e_robot", package_name="asbr_moveit")
        .robot_description(mappings={
            "robot_ip": "172.22.22.2",
            "use_fake_hardware": use_fake_hardware,
            "headless_mode": headless_mode,
            "com_port": com_port,
            "safety_limits": safety_limits,
            "safety_pos_margin": safety_pos_margin,
            "safety_k_position": safety_k_position,
        })
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )

     # MoveIt Configuration
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution(
                [FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(moveit_config_package), "config", moveit_config_file]
            )
        ]
    )
    robot_description_semantic = {"robot_description_semantic": ParameterValue(robot_description_semantic_content, value_type=str)}


    robot_description_kinematics = PathJoinSubstitution(
        [FindPackageShare(moveit_config_package), "config", "kinematics.yaml"]
    )

    robot_description_planning = {
        "robot_description_planning": load_yaml(
            str(moveit_config_package.perform(context)),
            os.path.join("config", str(moveit_joint_limits_file.perform(context))),
        )
    }
    
    # OMPL planning pipeline configuration
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization 
                                   default_planner_request_adapters/FixWorkspaceBounds 
                                   default_planner_request_adapters/FixStartStateBounds 
                                   default_planner_request_adapters/FixStartStateCollision 
                                   default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }
    # Load OMPL planning configuration
    ompl_planning_yaml = load_yaml("asbr_moveit", "config/ompl_planning.yaml")
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    # Load controllers yaml
    controllers_yaml = load_yaml("asbr_moveit", "config/controllers.yaml")
    moveit_controllers = {
        "moveit_simple_controller_manager": controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }


    # Move group node 
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            moveit_config.trajectory_execution,
            ompl_planning_pipeline_config,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {"use_sim_time": use_sim_time},
        ],
    )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "base_link"],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # ros2_control for trajectory execution 
    ros2_controller_path = PathJoinSubstitution([FindPackageShare("asbr_moveit"), "config", "ros2_controllers.yaml"])
    
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controller_path],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="both",
    )


    # RViz node
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(moveit_config_package), "config", "moveit.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            robot_description_kinematics,
            robot_description_planning,
        ],
    )

    nodes_to_start = [
        run_move_group_node, 
        rviz_node,
        static_tf,
        robot_state_publisher, 
        ros2_control_node,
        ]

    return nodes_to_start

def generate_launch_description():
    declared_arguments = []
    # UR specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur5",
            description="Type of UR robot to use."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Indicate whether the robot is running with fake hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_limits",
            default_value="true",
            description="Enables the safety limits controller if true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_pos_margin",
            default_value="0.15",
            description="The margin to lower and upper limits in the safety controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_k_position",
            default_value="20",
            description="k-position factor in the safety controller.",
        )
    )
    # General arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="asbr_description",
            description="Description package with robot URDF/XACRO files.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="asbr.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_package",
            default_value="asbr_moveit",
            description="MoveIt config package with robot SRDF/XACRO files.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_file",
            default_value="ur5e_robot.srdf",
            description="MoveIt SRDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_joint_limits_file",
            default_value="joint_limits.yaml",
            description="MoveIt joint limits that augment or override the values from the URDF robot_description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Make MoveIt use simulation time. This is needed for trajectory planning in simulation.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "com_port",
            default_value="/dev/ttyUSB0",
            description="Communication port for the gripper",
        ),
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "headless_mode",
            # Set to true if using the real UR5x
            default_value="true",
            description="Enable headless mode for real robot control.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for multi-robot setup.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?")
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])