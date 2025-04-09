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
    namespace: str | launch.substitution.Substitution | None = None,
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
    create_desc_file = f"/tmp/{create_exec_name}.yaml"
    create_desc_proc = launch.actions.ExecuteProcess(
        cmd=[
            "echo",
            [
                f'"- {gz_prefix}_topic_name: ',
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
                f'\n"',
            ],
            f"> {create_desc_file}",
        ],
        shell=True,
        name=create_exec_name,
    )
    bridge = launch_ros.actions.Node(
        package=ros_gz_bridge_pkg_name,
        executable="parameter_bridge",
        parameters=[{"config_file": create_desc_file}],
        namespace=namespace,
    )
    return [
        create_desc_proc,
        launch.actions.RegisterEventHandler(
            launch.event_handlers.OnProcessExit(
                target_action=create_desc_proc,
                on_exit=bridge,
            )
        ),
    ]


def generate_controller_file(
    yumi_launch_path: launch.substitution.Substitution,
    template_name: launch.substitution.Substitution,
    controller_name: str | launch.substitution.Substitution,
    controller_tmp_args: str | launch.substitution.Substitution,
    namespace: str | launch.substitution.Substitution = "",
) -> launch.actions.ExecuteProcess:
    generate_script_path = launch.substitutions.PathJoinSubstitution(
        [yumi_launch_path, "scripts", "generate_controller_file.py"]
    )
    template_path = launch.substitutions.PathJoinSubstitution(
        [yumi_launch_path, "config_templates", template_name]
    )
    create_desc_proc = launch.actions.ExecuteProcess(
        cmd=[
            "python3 ",
            generate_script_path,
            "-t",
            template_path,
            "-v",
            controller_tmp_args,
            [
                "namespace=",
                namespace,
                "/" if namespace else "",
            ],
            ">",
            [
                "/tmp/",
                controller_name,
                ".yaml",
            ],
        ],
        shell=True,
        name="generate_controller_config",
    )
    return create_desc_proc


def generate_launch_description():
    ld = launch.LaunchDescription()

    # Parameters
    is_ign = False
    yumi_launch_pkg_name = "yumi-launch"
    yumi_desc_pkg_name = "yumi-description"
    ros_gz_pkg_name = get_ros_gz_pkg_name(is_ign)
    ros_gz_launch_file = get_ros_gz_launch_file(is_ign)
    roz_gs_args_name = get_roz_gs_args_name(is_ign)

    # Arguments values
    robot_description = launch.substitutions.LaunchConfiguration("robot_description")
    arms_interface = launch.substitutions.LaunchConfiguration("arms_hardware_interface")
    body_name = launch.substitutions.LaunchConfiguration("body_name")
    use_sim_time = launch.substitutions.LaunchConfiguration("use_sim_time")
    ns = launch.substitutions.LaunchConfiguration("namespace")
    controller_update_rate = launch.substitutions.LaunchConfiguration(
        "controller_update_rate",
    )
    controller_tmp_args = launch.substitutions.LaunchConfiguration(
        "controller_tmp_args",
    )
    controller_name = launch.substitutions.LaunchConfiguration("controller_name")
    template_name = launch.substitutions.LaunchConfiguration("template_name")
    collision_enabled = launch.substitutions.LaunchConfiguration("collision_enabled")
    max_effort = launch.substitutions.LaunchConfiguration("max_effort")

    # Paths
    yumi_desc_pkg_path = launch_ros.substitutions.FindPackageShare(yumi_desc_pkg_name)
    yumi_launch_pkg_path = launch_ros.substitutions.FindPackageShare(
        yumi_launch_pkg_name
    )
    yumi_model_path = launch.substitutions.PathJoinSubstitution(
        [yumi_desc_pkg_path, "urdf", robot_description]
    )
    yumi_urdf = launch_ros.descriptions.ParameterValue(
        value=launch.substitutions.Command(
            command=[
                "xacro",
                " ",
                yumi_model_path,
                " ",
                "arms_interface:=",
                arms_interface,
                " ",
                "body_name:=",
                body_name,
                " ",
                "namespace:=",
                ns,
                " ",
                "collision_enabled:=",
                collision_enabled,
                " ",
                "max_effort:=",
                max_effort,
                " ",
                "controller_path:=/tmp/",
                controller_name,
                ".yaml",
            ]
        ),
        value_type=str,
    )
    ros_gz_path = launch_ros.substitutions.FindPackageShare(ros_gz_pkg_name)
    ros_gz_launch_path = launch.substitutions.PathJoinSubstitution(
        [ros_gz_path, "launch", ros_gz_launch_file]
    )
    robot_description_topic = launch.substitutions.PathJoinSubstitution(
        ["/", ns, "robot_description"]
    )
    controller_manager_path = launch.substitutions.PathJoinSubstitution(
        [ns, "controller_manager"]
    )

    # # Add environment variables for Gazebo
    # gazebo_env_setup = launch.actions.SetEnvironmentVariable(
    #     name="MESA_GL_VERSION_OVERRIDE", value="3.3"
    # )
    # # Enable software rendering
    # gazebo_sw_rendering = launch.actions.SetEnvironmentVariable(
    #     name="LIBGL_ALWAYS_SOFTWARE", value="1"
    # )
    # ld.add_action(gazebo_env_setup)
    # ld.add_action(gazebo_sw_rendering)

    # Define arguments
    ld.add_action(
        launch.actions.DeclareLaunchArgument(
            "namespace",
            default_value="yumi",
            description="Namespace for the robot",
        ),
    )
    ld.add_action(
        launch.actions.DeclareLaunchArgument(
            "arms_hardware_interface",
            default_value="EffortJointInterface",
            description="Hardware interface for the arms",
        ),
    )
    ld.add_action(
        launch.actions.DeclareLaunchArgument(
            "body_name",
            default_value="yumi",
            description="Name for base body",
        ),
    )
    ld.add_action(
        launch.actions.DeclareLaunchArgument(
            "collision_enabled",
            default_value="false",
            description="Enable collision during simulation",
        ),
    )
    ld.add_action(
        launch.actions.DeclareLaunchArgument(
            "max_effort",
            default_value="300",
            description="Max allowed effort",
        ),
    )
    ld.add_action(
        launch.actions.DeclareLaunchArgument(
            "robot_description",
            default_value="yumi.urdf.xacro",
            description="Robot description xacro",
        ),
    )
    ld.add_action(
        launch.actions.DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation (Gazebo) clock if true",
        ),
    )
    ld.add_action(
        launch.actions.DeclareLaunchArgument(
            "controller_update_rate",
            default_value="1000",
            description="Controller update rate",
        ),
    )
    ld.add_action(
        launch.actions.DeclareLaunchArgument(
            "controller_name",
            default_value="effort_controller",
            description="Controller name",
        ),
    )
    ld.add_action(
        launch.actions.DeclareLaunchArgument(
            "controller_tmp_args",
            default_value=[
                "update_rate=",
                controller_update_rate,
                " ",
                "body_name=",
                body_name,
                " ",
                "robot_description_topic=",
                robot_description_topic,
            ],
            description="Controller template substitutions args",
        ),
    )
    ld.add_action(
        launch.actions.DeclareLaunchArgument(
            "template_name",
            default_value=[controller_name, ".yaml.tmp"],
            description="Template file name",
        ),
    )

    # Define nodes
    node_robot_state_publisher = launch_ros.actions.Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": yumi_urdf,
                "use_sim_time": use_sim_time,
            },
        ],
        namespace=ns,
    )
    gz_spawn_entity = launch_ros.actions.Node(
        package=ros_gz_pkg_name,
        executable="create",
        output="screen",
        arguments=["-topic", robot_description_topic, "-name", body_name],
        namespace=ns,
    )
    controller_file_node = generate_controller_file(
        yumi_launch_path=yumi_launch_pkg_path,
        template_name=template_name,
        controller_name=controller_name,
        controller_tmp_args=controller_tmp_args,
        namespace=ns,
    )
    load_joint_state_broadcaster = launch.actions.ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "-c",
            controller_manager_path,
            "joint_state_broadcaster",
        ],
        output="screen",
    )
    load_chosen_controller = launch.actions.ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "-c",
            controller_manager_path,
            controller_name,
        ],
        output="screen",
    )
    nodes = launch.actions.GroupAction(
        [
            *generate_bridge(
                is_ign,
                "/clock",
                "/clock",
                "rosgraph_msgs/msg/Clock",
                "gz.msgs.Clock",
                GZ2ROS,
                "clock_bridge",
                namespace=ns,
            ),
            launch.actions.IncludeLaunchDescription(
                launch_description_source=ros_gz_launch_path,
                launch_arguments=[(roz_gs_args_name, ["-r empty.sdf"])],
            ),
            launch.actions.RegisterEventHandler(
                launch.event_handlers.OnProcessExit(
                    target_action=gz_spawn_entity,
                    on_exit=load_joint_state_broadcaster,
                )
            ),
            launch.actions.RegisterEventHandler(
                launch.event_handlers.OnProcessExit(
                    target_action=load_joint_state_broadcaster,
                    on_exit=load_chosen_controller,
                )
            ),
            launch.actions.RegisterEventHandler(
                launch.event_handlers.OnProcessExit(
                    target_action=controller_file_node,
                    on_exit=node_robot_state_publisher,
                )
            ),
            launch.actions.RegisterEventHandler(
                launch.event_handlers.OnProcessExit(
                    target_action=controller_file_node,
                    on_exit=gz_spawn_entity,
                )
            ),
            controller_file_node,
        ]
    )
    ld.add_action(nodes)

    return ld
