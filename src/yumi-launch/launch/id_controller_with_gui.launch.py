#!/usr/bin/env python3

import launch
import launch.event_handlers
import launch.launch_description_sources
import launch.substitutions
import launch_ros
import launch_ros.substitutions


def generate_launch_description():
    ld = launch.LaunchDescription()

    # Parameters
    yumi_launch_pkg_name = "yumi-launch"
    controller_name_value = "id_controller"
    template_name_value = "id_controller.yaml.tmp"

    # Arguments values
    ns = launch.substitutions.LaunchConfiguration("namespace")

    # Paths
    yumi_launch_pkg_path = launch_ros.substitutions.FindPackageShare(
        yumi_launch_pkg_name
    )
    base_launch_file = launch.substitutions.PathJoinSubstitution(
        [yumi_launch_pkg_path, "launch", "base.launch.py"]
    )

    # Define arguments
    ld.add_action(
        launch.actions.DeclareLaunchArgument(
            "namespace",
            default_value="yumi",
            description="Namespace for the robot",
        ),
    )

    # Define nodes
    joint_state_publisher_gui_node = launch_ros.actions.Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen",
        namespace=ns,
        remappings=[
            ("joint_states", "id_controller/commands"),
        ],
    )

    # Include base launch file
    include_base_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            base_launch_file
        ),
        launch_arguments={
            "namespace": ns,
            "controller_name": controller_name_value,
            "template_name": template_name_value,
        }.items(),
    )

    # Add actions to the launch description
    ld.add_action(include_base_launch)
    ld.add_action(joint_state_publisher_gui_node)

    return ld
