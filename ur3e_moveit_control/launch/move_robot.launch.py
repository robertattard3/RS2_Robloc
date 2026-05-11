from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription, OpaqueFunction, TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ur_onrobot_moveit_config.launch_common import load_yaml


def launch_setup(context):

    robot_ip        = LaunchConfiguration("robot_ip")
    camera_topic    = LaunchConfiguration("camera_topic")
    use_fake        = LaunchConfiguration("use_fake_hardware").perform(context)
    use_camera      = LaunchConfiguration("use_camera").perform(context).lower() == "true"
    is_simulation   = use_fake.lower() == "true"

    # ── Robot description (URDF) ─────────────────────────────────────────────
    # Identical to what ur_onrobot_moveit.launch.py builds for move_group.
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([
            FindPackageShare("ur_onrobot_description"), "urdf", "ur_onrobot.urdf.xacro"
        ]),
        " robot_ip:=xxx.yyy.zzz.www",
        " joint_limit_params:=",
        PathJoinSubstitution([FindPackageShare("ur_description"), "config", "ur3e", "joint_limits.yaml"]),
        " kinematics_params:=",
        PathJoinSubstitution([FindPackageShare("ur_description"), "config", "ur3e", "default_kinematics.yaml"]),
        " physical_params:=",
        PathJoinSubstitution([FindPackageShare("ur_description"), "config", "ur3e", "physical_parameters.yaml"]),
        " visual_params:=",
        PathJoinSubstitution([FindPackageShare("ur_description"), "config", "ur3e", "visual_parameters.yaml"]),
        " safety_limits:=true",
        " safety_pos_margin:=0.15",
        " safety_k_position:=20",
        " name:=ur_onrobot",
        " ur_type:=ur3e",
        " onrobot_type:=rg2",
        " script_filename:=ros_control.urscript",
        " input_recipe_filename:=rtde_input_recipe.txt",
        " output_recipe_filename:=rtde_output_recipe.txt",
        ' prefix:=""',
    ])
    robot_description = {"robot_description": robot_description_content}

    # ── Robot description semantic (SRDF) ────────────────────────────────────
    robot_description_semantic_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([
            FindPackageShare("ur_onrobot_moveit_config"), "srdf", "ur_onrobot.srdf.xacro"
        ]),
        " name:=ur_onrobot",
        ' prefix:=""',
    ])
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content}

    # ── MoveIt planning config ───────────────────────────────────────────────
    robot_description_kinematics = {
        "robot_description_kinematics": load_yaml(
            "ur_onrobot_moveit_config", "config/kinematics.yaml"
        )
    }
    ompl_planning_yaml = load_yaml("ur_onrobot_moveit_config", "config/ompl_planning.yaml")
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": (
                "default_planner_request_adapters/AddTimeOptimalParameterization "
                "default_planner_request_adapters/FixWorkspaceBounds "
                "default_planner_request_adapters/FixStartStateBounds "
                "default_planner_request_adapters/FixStartStateCollision "
                "default_planner_request_adapters/FixStartStatePathConstraints"
            ),
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    # ── Hardware + MoveIt launches ───────────────────────────────────────────
    ur_onrobot_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("ur_onrobot_control"), "launch", "start_robot.launch.py"
            ])
        ]),
        launch_arguments={
            "ur_type":                    "ur3e",
            "onrobot_type":               "rg2",
            "robot_ip":                   robot_ip,
            "use_fake_hardware":          use_fake,
            "launch_rviz":                "false",
            "controller_spawner_timeout": "60",
            "reverse_port":               "50002",
        }.items(),
    )

    ur_onrobot_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("ur_onrobot_moveit_config"), "launch", "ur_onrobot_moveit.launch.py"
            ])
        ]),
        launch_arguments={
            "ur_type":      "ur3e",
            "onrobot_type": "rg2",
            "launch_rviz":  "true",
            "launch_servo": "false",
        }.items(),
    )

    # ── Static transforms: camera position relative to base_link ────────────
    # Edit x/y/z/roll/pitch/yaw to match your physical camera mounting.
    camera_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_static_tf",
        arguments=[
            "--x",     "0.620", # PHYSICAL MEASUREMENT: z=330, x=480, y=285
            "--y",     "0.275",
            "--z",     "0.330",
            "--roll",  "0.0",
            "--pitch", "0.4363",  # 25° down from horizontal
            "--yaw",   "3.14159",  # π rad: image "up" faces toward robot
            "--frame-id",       "base_link",
            "--child-frame-id", "camera_link",
        ],
    )

    # Standard RealSense optical-frame rotation: camera_link -> camera_color_optical_frame.
    # Replaces the TF frames normally published by the realsense2_camera driver.
    camera_optical_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_optical_static_tf",
        arguments=[
            "--x",     "0.0",
            "--y",     "0.0",
            "--z",     "0.0",
            "--roll",  "-1.5708",   # -π/2
            "--pitch", "0.0",
            "--yaw",   "-1.5708",   # -π/2
            "--frame-id",       "camera_link",
            "--child-frame-id", "camera_color_optical_frame",
        ],
    )

    # ── Delayed nodes at 15 s ────────────────────────────────────────────────
    delayed_actions = [
        Node(
            package="ur3e_moveit_control",
            executable="pick_place_mtc",
            output="screen",
            parameters=[
                robot_description,
                robot_description_semantic,
                robot_description_kinematics,
                ompl_planning_pipeline_config,
            ],
        ),
        Node(
            package="ur3e_moveit_control",
            executable="eef_pose_publisher.py",
            output="screen",
        ),
    ]

    if is_simulation:
        delayed_actions.append(Node(
            package="ur3e_moveit_control",
            executable="fake_finger_width_state.py",
            output="screen",
        ))

    delayed_nodes = TimerAction(period=15.0, actions=delayed_actions)

    ur3e_ui = ExecuteProcess(
        cmd=["ros2", "run", "rqt_gui", "rqt_gui",
             "--standalone", "UR3eUIPlugin"],
        output="screen",
    )

    actions = [
        ur_onrobot_control_launch,
        ur_onrobot_moveit_launch,
        ur3e_ui,
        delayed_nodes,
    ]

    if use_camera:
        actions += [
            camera_tf,
            camera_optical_tf,
            TimerAction(
                period=15.0,
                actions=[
                    Node(
                        package="ur3e_moveit_control",
                        executable="camera_to_robot_goals.py",
                        output="screen",
                        parameters=[{
                            "input_topic":  camera_topic,
                            "target_frame": "base_link",
                        }],
                    ),
                ]
            ),
        ]
        if not is_simulation:
            actions.append(Node(
                package="cube_detector",
                executable="live_detect",
                output="log",
            ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="true",
            description="Use simulated hardware (true) or real robot (false)",
        ),
        DeclareLaunchArgument(
            "use_camera",
            default_value="false",
            description="Launch camera TF, cube detector and goal bridge (true) or skip them (false)",
        ),
        DeclareLaunchArgument(
            "camera_topic",
            default_value="/cube_poses",
            description="Topic where the camera publishes goal poses (PoseArray in camera frame)",
        ),
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.56.101",
            description="IP address of the UR robot or URSim instance",
        ),
        OpaqueFunction(function=launch_setup),
    ])
