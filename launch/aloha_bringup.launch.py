import yaml

from interbotix_xs_modules.xs_launch import (
    declare_interbotix_xsarm_robot_description_launch_arguments,
)
from interbotix_common_modules.launch import (
    AndCondition,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.conditions import (
    IfCondition,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterFile


def load_yaml_file(yaml_path):
    # Function to read and parse the YAML file
    with open(yaml_path, "r") as f:
        return yaml.safe_load(f)


def launch_setup(context, *args, **kwargs):

    nodes = []

    # Get the base name from the 'robot_config_name' launch argument and construct the YAML path
    robot_config_base = LaunchConfiguration("robot").perform(context)
    yaml_file_path = PathJoinSubstitution([
        FindPackageShare("aloha"),
        "config",
        "robot",
        TextSubstitution(text=f"{robot_config_base}.yaml"),
    ]).perform(context)

    # Load the YAML file
    config = load_yaml_file(yaml_file_path).get('robot', {})

    descriptions = {}

    # Leader arms
    for leader in config.get("leader_arms", []):
        descriptions[leader["name"]] = LaunchConfiguration(
            f'robot_description_{leader["name"]}'
        )

    # Follower arms
    for follower in config.get("follower_arms", []):
        descriptions[follower["name"]] = LaunchConfiguration(
            f'robot_description_{follower["name"]}'
        )

    # Leader arms
    for leader in config.get("leader_arms", []):

        xsarm_control_leader_launch_include = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare("interbotix_xsarm_control"),
                    "launch",
                    "xsarm_control.launch.py",
                ])
            ]),
            launch_arguments={
                "robot_model": leader["model"],
                "robot_name": leader["name"],
                "mode_configs": PathJoinSubstitution([
                    FindPackageShare("aloha"),
                    "config",
                    f"leader_modes_{leader['orientation']}.yaml",
                ]),
                "motor_configs": PathJoinSubstitution([
                    FindPackageShare("interbotix_xsarm_control"),
                    "config",
                    f"{leader['model']}.yaml",
                ]),
                "use_rviz": "false",
                "robot_description": Command([
                    FindExecutable(name="xacro"),
                    " ",
                    PathJoinSubstitution([
                        FindPackageShare("interbotix_xsarm_descriptions"),
                        "urdf",
                        f'{leader["model"]}.urdf.xacro',
                    ]),
                    " ",
                    "robot_name:=",
                    leader["name"],
                ]),
            }.items(),
            condition=IfCondition(LaunchConfiguration("launch_leaders")),
        )

        nodes.append(xsarm_control_leader_launch_include)

        # Add transform broadcaster for leader arm
        leader_transform_broadcaster = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name=f'{leader["name"]}_transform_broadcaster',
            arguments=[
                *map(str, leader["transform"]),  # Transform values from YAML
                "/world",                        # Parent frame
                f'/{leader["name"]}/base_link',  # Child frame
            ],
        )
        nodes.append(leader_transform_broadcaster)

        # Add gravity compensation node if enabled
        gravity_compensation_node = Node(
            package="interbotix_gravity_compensation",
            executable="interbotix_gravity_compensation",
            name="gravity_compensation",
            namespace=leader["name"],
            parameters=[
                {
                    "motor_specs": PathJoinSubstitution([
                        FindPackageShare("aloha"),
                        "config",
                        f"leader_motor_specs_{leader['orientation']}.yaml",
                    ])
                }
            ],
            output="screen",
            condition=AndCondition([
                IfCondition(LaunchConfiguration("use_gravity_compensation")),
                IfCondition(LaunchConfiguration("launch_leaders"))
            ]),
        )
        nodes.append(gravity_compensation_node)

    # Follower arms
    for follower in config.get("follower_arms", []):
        # Launch follower arm node
        xsarm_control_follower_launch_include = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare("interbotix_xsarm_control"),
                    "launch",
                    "xsarm_control.launch.py",
                ])
            ]),
            launch_arguments={
                "robot_model": follower["model"],
                "robot_name": follower["name"],
                "mode_configs": PathJoinSubstitution([
                    FindPackageShare("aloha"),
                    "config",
                    f"follower_modes_{follower['orientation']}.yaml",
                ]),
                "motor_configs": PathJoinSubstitution([
                    FindPackageShare("interbotix_xsarm_control"),
                    "config",
                    f"{follower['model']}.yaml",
                ]),
                "use_rviz": "false",
                "robot_description": Command([
                    FindExecutable(name="xacro"),
                    " ",
                    PathJoinSubstitution([
                        FindPackageShare("interbotix_xsarm_descriptions"),
                        "urdf",
                        f'{follower["model"]}.urdf.xacro',
                    ]),
                    " ",
                    "robot_name:=",
                    follower["name"],
                ]),
            }.items(),
            condition=IfCondition(
                LaunchConfiguration("launch_followers")
            ),
        )

        nodes.append(xsarm_control_follower_launch_include)

        # Add transform broadcaster for follower arm
        follower_transform_broadcaster = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name=f'{follower["name"]}_transform_broadcaster',
            arguments=[
                *map(str, follower["transform"]),  # Transform values from YAML
                "/world",                          # Parent frame
                f'/{follower["name"]}/base_link',  # Child frame
            ],
        )

        nodes.append(follower_transform_broadcaster)

    arms_group_action = GroupAction(
        actions=nodes,
    )

    rs_actions = []

    # Load the common parameters once from the YAML
    common_params = config.get("cameras", {}).get("common_parameters", {})

    # Iterate through the individual cameras defined in the YAML
    for camera in config.get("cameras", {}).get("camera_instances", []):
        # Merge common parameters with camera-specific settings
        camera_params = common_params.copy()
        camera_params.update(
            {
                # Serial number or default
                "serial_no": camera.get("serial_no", ""),
                # Reset flag, default to True
                "initial_reset": camera.get("initial_reset", True),
            }
        )

        # Create a node for each camera with parameters from YAML
        rs_actions.append(
            Node(
                package="realsense2_camera",
                namespace=camera["name"],  # Namespace based on camera name
                name="camera",
                executable="realsense2_camera_node",
                parameters=[camera_params],  # Apply merged parameters
                output="screen",
            )
        )

    # Create the GroupAction to conditionally include all the camera nodes
    realsense_ros_launch_includes_group_action = GroupAction(
        condition=IfCondition(LaunchConfiguration("use_cameras")),
        actions=rs_actions,
    )

    base_nodes = []

    # Check if base is enabled in the YAML configuration
    if config.get("base", False):
        # Launch the base nodes
        slate_base_node = Node(
            package="interbotix_slate_driver",
            executable="slate_base_node",
            name="slate_base",
            output="screen",
            namespace="mobile_base",
        )

        joystick_teleop_node = Node(
            package="teleop_twist_joy",
            executable="teleop_node",
            name="base_joystick_teleop",
            namespace="mobile_base",
            parameters=[
                ParameterFile(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("aloha"),
                            "config",
                            "base_joystick_teleop.yaml",
                        ]
                    ),
                    allow_substs=True,
                ),
            ],
        )

        joy_node = Node(
            package="joy",
            executable="joy_node",
            name="joy_node",
            namespace="mobile_base",
            parameters=[
                {
                    "dev": "/dev/input/js0",
                    "deadzone": 0.3,
                    "autorepeat_rate": 20.0,
                }
            ],
        )

        # Add the base-related nodes to your launch actions list
        base_nodes.extend([slate_base_node, joystick_teleop_node, joy_node])

    else:
        print("Base is disabled. Skipping base node launches.")

    base_group_action = GroupAction(
        actions=base_nodes,
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("aloha_rvizconfig")],
        condition=IfCondition(LaunchConfiguration("use_aloha_rviz")),
    )

    loginfo_action = LogInfo(
        msg=[
            "\n==== Launching ALOHA Robot Setup ====\n",
            "\nRobot Configuration:",
            "\n- Selected robot configuration file: ", LaunchConfiguration(
                "robot"),
            "\n\nArm Launch Configurations:",
            "\n- Launch leader arms: ", LaunchConfiguration("launch_leaders"),
            "\n- Launch follower arms: ", LaunchConfiguration(
                "launch_followers"),

            "\n\nCamera Configuration:",
            "\n- Enable cameras: ", LaunchConfiguration("use_cameras"),
            "\n- Cameras to be launched (from configuration): ", ", ".join(
                [camera["name"] for camera in config.get(
                    "cameras", {}).get("camera_instances", [])]
            ),

            "\n\nBase Configuration:",
            "\n- Base enabled: ", "true" if config.get(
                "base", False) else "false",

            "\n\nAdditional Configurations:",
            "\n- Use gravity compensation: ", LaunchConfiguration(
                "use_gravity_compensation"),
            "\n- Use RViz for visualization: ", LaunchConfiguration(
                "use_aloha_rviz"),
            "\n- RViz configuration file: ", LaunchConfiguration(
                "aloha_rvizconfig"),

            "\n\n==== End of Launch Configuration ====\n"
        ]
    )

    return [
        arms_group_action,
        realsense_ros_launch_includes_group_action,
        base_group_action,
        rviz2_node,
        loginfo_action,
    ]


def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "robot",
            default_value="aloha_stationary",
            description="Name of the robot configuration file (e.g., aloha_stationary, aloha_solo, aloha_mobile)",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_leaders",
            default_value="true",
            choices=("true", "false"),
            description=(
                "if `true`, launches both the leader and follower arms; if `false, just the "
                "followers are launched"
            ),
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_followers",
            default_value="true",
            choices=("true", "false"),
            description=(
                "if `true`, launches both the leader and follower arms; if `false, just the "
                "leaders are launched"
            ),
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_cameras",
            default_value="true",
            choices=("true", "false"),
            description="if `true`, launches the camera drivers.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_aloha_rviz",
            default_value="false",
            choices=("true", "false"),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "aloha_rvizconfig",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("aloha"),
                    "rviz",
                    "aloha.rviz",
                ]
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_gravity_compensation",
            default_value="true",
            choices=("true", "false"),
            description="if `true`, launches the gravity compensation node",
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
