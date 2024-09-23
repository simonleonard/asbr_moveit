# Developer: Seyi R. Afolayan 
# Credits: Denis Stogl 

import os 

from launch_ros.actions import Node 
from launch_ros.substitutions import FindPackageShare
from ur_moveit_config.launch_common import load_yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, FindExecutable

def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur5",
            description="Type/Series of the robot used.",
        ),
        DeclareLaunchArgument(
            "use_fake_hardware", 
            default_value="true",
            description="Initially loaded robot controller",
        ),
        DeclareLaunchArgument(
            "safety_limits", 
            default_value="true",
        ),
        DeclareLaunchArgument(
            "safety_pos_margin", 
            default_value="0.15",
        ),
        DeclareLaunchArgument(
            "safety_k_position",
            default_value="20",
        ),
        DeclareLaunchArgument(
            "description_package", 
            default_value="asbr_description",
        ),
        DeclareLaunchArgument(
            "description_file",
            default_value="asbr.urdf.xacro",
        ),
        DeclareLaunchArgument(
            "moveit_config_package",
            default_value="asbr_moveit",
        ),
        DeclareLaunchArgument(
            "moveit_joint_limits_file",
            default_value="joint_limits.yaml",
        ),
        DeclareLaunchArgument(
            "moveit_config_file",
            default_value="ur5e_robot.srdf",
        ),
        DeclareLaunchArgument(
            "prefix",
            default_value="",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
        ),
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="false",
        ),
    ]


    # Initializations
    ur_type = LaunchConfiguration("ur_type") 
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    moveit_config_package = LaunchConfiguration("moveit_config_package")
    moveit_config_file = LaunchConfiguration("moveit_config_file")
    moveit_joint_limits_file = LaunchConfiguration("moveit_joint_limits_file")
    prefix = LaunchConfiguration("prefix")
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_rviz = LaunchConfiguration("launch_rviz")

    # Additional stuffs from the config package 
    joint_limit_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", ur_type, "joint_limits.yaml"]
    )
    kinematics_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", ur_type, "default_kinematics.yaml"]
    )
    physical_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", ur_type, "physical_parameters.yaml"]
    )
    visual_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", ur_type, "visual_parameters.yaml"]
    )

    # Setting the robot description 
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]), 
            " ", 
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]), 
            " ",
            "joint_limit_params:=", joint_limit_params,
            " ",
            "kinematics_params:=", kinematics_params,
            " ",
            "physical_params:=", physical_params,
            " ",
            "visual_params:=", visual_params,
            " ",
            "safety_limits:=", safety_limits,
            " ",
            "safety_pos_margin:=", safety_pos_margin,
            " ",
            "safety_k_position:=", safety_k_position,
            " ", 
            "name:=", "ur5e_robot",
            " ",
            "ur_type:=", ur_type,
            " ",
            "prefix:=", prefix,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # Moveit Configuration 
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindPackageShare(moveit_config_package), "config", moveit_config_file]),
            " ",
            "name:=", "ur5e_robot",
            " ",
            "prefix:=", prefix,
            " ",
        ]
    )
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content}

    robot_description_kinematics = PathJoinSubstitution(
        [FindPackageShare(moveit_config_package), "config", "kinematics.yaml"]
    )

    def launch_setup(context):   
        robot_description_planning = {
            "robot_description_planning": load_yaml(
                str(moveit_config_package.perform(context)),
                os.path.join("config", str(moveit_joint_limits_file.perform(context)))
            )
        }

        # Planning Configuration
        ompl_planning_pipeline_config = {
            "move_group": {
                "planning_plugin": "ompl_interface/OMPLPlanner",
                "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
                "start_state_max_bounds_error": 0.1,
            }
        }
        ompl_planning_yaml = load_yaml("asbr_moveit", "config/ompl_planning.yaml")
        ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

        # Trajectory Execution Configuration
        controllers_yaml = load_yaml("ur_moveit_config", "config/controllers.yaml")
        # the scaled_joint_trajectory_controller does not work on fake hardware
        change_controllers = context.perform_substitution(use_fake_hardware)
        if change_controllers == "true":
            controllers_yaml["scaled_joint_trajectory_controller"]["default"] = False
            controllers_yaml["joint_trajectory_controller"]["default"] = True

        moveit_controllers = {
            "moveit_simple_controller_manager": controllers_yaml,
            "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
        }

        trajectory_execution = {
            "moveit_manage_controllers": False,
            "trajectory_execution.allowed_execution_duration_scaling": 1.2,
            "trajectory_execution.allowed_goal_duration_margin": 0.5,
            "trajectory_execution.allowed_start_tolerance": 0.01,
        }

        planning_scene_monitor_parameters = {
            "publish_planning_scene": True,
            "publish_geometry_updates": True,
            "publish_state_updates": True,
            "publish_transforms_updates": True,
        }
