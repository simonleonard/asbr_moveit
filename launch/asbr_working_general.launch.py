import os

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ur_moveit_config.launch_common import load_yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.parameter_descriptions import ParameterValue

def launch_setup(context, *args, **kwargs):

    # Initialize Arguments
    ur_type = LaunchConfiguration("ur_type")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")
    com_port = LaunchConfiguration("com_port")
    headless_mode = LaunchConfiguration("headless_mode")
    # General arguments
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    moveit_config_package = LaunchConfiguration("moveit_config_package")
    moveit_joint_limits_file = LaunchConfiguration("moveit_joint_limits_file")
    moveit_config_file = LaunchConfiguration("moveit_config_file")
    prefix = LaunchConfiguration("prefix")
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_rviz = LaunchConfiguration("launch_rviz")
    launch_servo = LaunchConfiguration("launch_servo")

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

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),
            " ",
            "robot_ip:=172.22.22.2",
            " ",
            "joint_limit_params:=",
            joint_limit_params,
            " ",
            "kinematics_params:=",
            kinematics_params,
            " ",
            "physical_params:=",
            physical_params,
            " ",
            "visual_params:=",
            visual_params,
            " ",
            "use_fake_hardware:=",
            use_fake_hardware,
            " ",
            "headless_mode:=",
            headless_mode,
            " ",
            "safety_limits:=",
            safety_limits,
            " ",
            "safety_pos_margin:=",
            safety_pos_margin,
            " ",
            "safety_k_position:=",
            safety_k_position,
            " ",
            "name:=",
            "ur",
            " ",
            "ur_type:=",
            ur_type,
            " ",
            "script_filename:=ros_control.urscript",
            " ",
            "input_recipe_filename:=rtde_input_recipe.txt",
            " ",
            "output_recipe_filename:=rtde_output_recipe.txt",
            " ",
            "prefix:=",
            prefix,
            " ",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # MoveIt Configuration
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution(
                [FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("asbr_moveit"), "config", moveit_config_file]
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

    # DEFAULT OMPL Planning Configuration
    # ompl_planning_pipeline_config = {
    #     "move_group": {
    #         "planning_plugin": "ompl_interface/OMPLPlanner",
    #         "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization 
    #                                default_planner_request_adapters/FixWorkspaceBounds 
    #                                default_planner_request_adapters/FixStartStateBounds 
    #                                default_planner_request_adapters/FixStartStateCollision 
    #                                default_planner_request_adapters/FixStartStatePathConstraints""",
    #         "start_state_max_bounds_error": 0.1,
    #     }
    # }
    # ompl_planning_yaml = load_yaml("asbr_moveit", "config/ompl_planning.yaml")
    # ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    # ASBR Planning Configuration
    asbr_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "assignment3/ASBRPlanner",
            "start_state_max_bounds_error": 0.1,
        }
    }
    asbr_planning_yaml = load_yaml("asbr_moveit", "config/asbr_planning.yaml")
    asbr_planning_pipeline_config["move_group"].update(asbr_planning_yaml)

    # Trajectory Execution Configuration
    controllers_yaml = load_yaml("asbr_moveit", "config/controllers.yaml")
    # change_controllers = context.perform_substitution(use_fake_hardware)
    # if change_controllers == "true":
    #     controllers_yaml["scaled_joint_trajectory_controller"]["default"] = False
    #     controllers_yaml["joint_trajectory_controller"]["default"] = True

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

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            #ompl_planning_pipeline_config,
            asbr_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {"use_sim_time": use_sim_time},
        ],
    )

    # rviz with moveit configuration
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
            robot_description,
            robot_description_semantic,
            #ompl_planning_pipeline_config,
            asbr_planning_pipeline_config,
            robot_description_kinematics,
            robot_description_planning,
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
        parameters=[robot_description],
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

    # Servo node for realtime control
    servo_yaml = load_yaml("asbr_moveit", "config/ur_servo.yaml")
    servo_params = {"moveit_servo": servo_yaml}
    servo_node = Node(
        package="moveit_servo",
        condition=IfCondition(launch_servo),
        executable="servo_node_main",
        parameters=[
            servo_params,
            robot_description,
            robot_description_semantic,
        ],
        output="screen",
    )

    nodes_to_start = [
        move_group_node, 
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
            default_value="ur5"
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
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for multi-robot setup.",
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
        DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?")
    )
    # declared_arguments.append(
    #     DeclareLaunchArgument("launch_servo", default_value="true", description="Launch Servo?")
    # )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])


#### The asbr planner was moving too fast and was failing due to tolerance violations
## Rectified by looking at the demo instructions to descrease the velocity to 0.01