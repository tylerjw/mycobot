#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = []
    # General arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="mycobot_280_description",
            description="Description package with robot URDF/XACRO files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="mycobot_280.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed, then also joint names in the controllers' configuration \
        have to be updated.",
        )
    )

    # Initialize Arguments
    mycobot_type = "mycobot_280"
    # General arguments
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    prefix = LaunchConfiguration("prefix")

    joint_limit_params = PathJoinSubstitution(
        [
            FindPackageShare(description_package),
            "config",
            mycobot_type,
            "joint_limits.yaml",
        ]
    )
    kinematics_params = PathJoinSubstitution(
        [
            FindPackageShare(description_package),
            "config",
            mycobot_type,
            "default_kinematics.yaml",
        ]
    )
    physical_params = PathJoinSubstitution(
        [
            FindPackageShare(description_package),
            "config",
            mycobot_type,
            "physical_parameters.yaml",
        ]
    )
    visual_params = PathJoinSubstitution(
        [
            FindPackageShare(description_package),
            "config",
            mycobot_type,
            "visual_parameters.yaml",
        ]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", description_file]
            ),
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
            "name:=",
            mycobot_type,
            " ",
            "prefix:=",
            prefix,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz", "view_robot.rviz"]
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    nodes_to_start = [
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
