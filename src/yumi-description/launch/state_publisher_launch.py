import launch
import launch.event_handlers
import launch.launch_description_sources
import launch.substitutions
import launch_ros
import launch_ros.substitutions


GZ2ROS = "["
ROS2GZ = "]"
BIDIRECTIONAL = "@"


def get_ros_gz_pkg_name(is_ign: bool) -> str:
    return "ros_gz_sim" if not is_ign else "ros_ign_gazebo"


def get_ros_gz_launch_file(is_ign: bool) -> str:
    return "gz_sim.launch.py" if not is_ign else "ign_gazebo.launch.py"


def get_roz_gs_args_name(is_ign: bool) -> str:
    return "gz_args" if not is_ign else "ign_args"


def get_ros_gz_bridge_pkg_name(is_ign: bool) -> str:
    return "ros_gz_bridge" if not is_ign else "ros_ign_bridge"


def generate_bridge(
    is_ign: bool,
    ros_topic: str | launch.substitution.Substitution,
    gz_topic: str | launch.substitution.Substitution,
    ros_msg: str,
    gz_msg: str,
    direction: str,
    bridge_name: str,
):
    ros_gz_bridge_pkg_name = get_ros_gz_bridge_pkg_name(is_ign)
    gz_prefix = "gz" if not is_ign else "ign"

    if is_ign:
        gz_msg = gz_msg.replace("gz.", "ignition.")

    if direction == GZ2ROS:
        direction = f"{gz_prefix.upper()}_TO_ROS"
    if direction == ROS2GZ:
        direction = f"ROS_TO_{gz_prefix.upper()}"

    create_exec_name = f"{bridge_name}_config"
    create_desc_file = f"{create_exec_name}.yaml"
    create_desc_proc = launch.actions.ExecuteProcess(
        cmd=[
            "echo",
            ' "',
            f"- {gz_prefix}_topic_name: ",
            gz_topic,
            "\n",
            f"  {gz_prefix}_type_name: ",
            gz_msg,
            "\n",
            "  ros_topic_name: ",
            ros_topic,
            "\n",
            "  ros_type_name: ",
            ros_msg,
            "\n",
            "  direction: ",
            direction,
            f'\n" >> {create_desc_file}',
        ],
        shell=True,
        name=create_exec_name,
    )
    bridge = launch_ros.actions.Node(
        package=ros_gz_bridge_pkg_name,
        executable="parameter_bridge",
        parameters=[{"config_file": create_desc_file}],
    )
    return [
        create_desc_proc,
        bridge,
        # launch.actions.RegisterEventHandler(
        #     launch.event_handlers.OnExecutionComplete(
        #         target_action=create_desc_proc,
        #         on_completion=bridge,
        #     )
        # ),
    ]


def generate_launch_description():
    ld = launch.LaunchDescription()

    # ctx = launch.LaunchContext()
    is_ign = False
    yumi_pkg_name = "yumi-description"
    ros_gz_pkg_name = get_ros_gz_pkg_name(is_ign)
    ros_gz_launch_file = get_ros_gz_launch_file(is_ign)
    roz_gs_args_name = get_roz_gs_args_name(is_ign)

    robot_description = launch.substitutions.LaunchConfiguration("robot_description")
    arms_interface = launch.substitutions.LaunchConfiguration("arms_hardware_interface")
    grippers_interface = launch.substitutions.LaunchConfiguration(
        "grippers_hardware_interface"
    )
    rviz_config_path = launch.substitutions.LaunchConfiguration("rvizconfig")
    use_sim_time = launch.substitutions.LaunchConfiguration("use_sim_time")
    ns = launch.substitutions.LaunchConfiguration("namespace")

    yumi_package_path = launch_ros.substitutions.FindPackageShare(yumi_pkg_name)
    yumi_model_path = launch.substitutions.PathJoinSubstitution(
        [yumi_package_path, "urdf", robot_description]
    )
    yumi_urdf = (
        launch.substitutions.Command(
            [
                "xacro",
                " ",
                yumi_model_path,
                " ",
                "arms_interface:=",
                arms_interface,
                " ",
                "grippers_interface:=",
                grippers_interface,
            ]
        ),
    )
    default_rviz_config_path = launch.substitutions.PathJoinSubstitution(
        [yumi_package_path, "urdf", "yumi.rviz"]
    )
    ros_gz_path = launch_ros.substitutions.FindPackageShare(ros_gz_pkg_name)
    ros_gz_launch_path = launch.substitutions.PathJoinSubstitution(
        [ros_gz_path, "launch", ros_gz_launch_file]
    )
    gz_joint_states = launch.substitutions.PathJoinSubstitution(
        [
            "/world/empty/model",
            ns,
            "joint_state",
        ]
    )

    ld.add_action(
        launch.actions.DeclareLaunchArgument(
            "namespace", default_value="yumi", description="Namespace for the robot"
        )
    )
    ld.add_action(
        launch.actions.DeclareLaunchArgument(
            "arms_hardware_interface",
            default_value="PositionJointInterface",
            description="Hardware interface for the arms",
        )
    )
    ld.add_action(
        launch.actions.DeclareLaunchArgument(
            "grippers_hardware_interface",
            default_value="EffortJointInterface",
            description="Hardware interface for the grippers",
        )
    )
    ld.add_action(
        launch.actions.DeclareLaunchArgument(
            "robot_description",
            default_value="yumi.urdf.xacro",
            description="Robot description xacro",
        )
    )
    ld.add_action(
        launch.actions.DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation (Gazebo) clock if true",
        ),
    )
    ld.add_action(
        launch.actions.DeclareLaunchArgument(
            name="rvizconfig",
            default_value=default_rviz_config_path,
            description="Absolute path to rviz config file",
        )
    )

    nodes = launch.actions.GroupAction(
        [
            launch_ros.actions.Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[
                    {
                        "robot_description": launch_ros.descriptions.ParameterValue(
                            value=yumi_urdf,
                            value_type=str,
                        )
                    },
                ],
            ),
            launch.actions.IncludeLaunchDescription(
                launch_description_source=ros_gz_launch_path,
                launch_arguments=[(roz_gs_args_name, "empty.sdf")],
            ),
            launch_ros.actions.Node(
                package=ros_gz_pkg_name,
                executable="create",
                arguments=["-topic", "robot_description", "-name", ns],
            ),
            *generate_bridge(
                is_ign,
                "/joint_states",
                gz_joint_states,
                "sensor_msgs/msg/JointState",
                "gz.msgs.Model",
                GZ2ROS,
                "joint_states",
            ),
        ]
    )
    ld.add_action(nodes)

    return ld
