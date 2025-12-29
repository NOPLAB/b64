# SPDX-FileCopyrightText: 2025 nop
# SPDX-License-Identifier: MIT

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    input_topic_arg = DeclareLaunchArgument(
        'input_topic',
        default_value='/input',
        description='Input topic for encoder'
    )

    input_type_arg = DeclareLaunchArgument(
        'input_type',
        default_value='std_msgs/msg/String',
        description='Message type for input topic'
    )

    output_topic_arg = DeclareLaunchArgument(
        'output_topic',
        default_value='/output',
        description='Output topic for decoder'
    )

    output_type_arg = DeclareLaunchArgument(
        'output_type',
        default_value='std_msgs/msg/String',
        description='Message type for output topic'
    )

    encode_node = Node(
        package='b64',
        executable='b64_encode',
        name='b64_encode',
        parameters=[{
            'input_topic': LaunchConfiguration('input_topic'),
            'input_type': LaunchConfiguration('input_type'),
        }],
    )

    decode_node = Node(
        package='b64',
        executable='b64_decode',
        name='b64_decode',
        parameters=[{
            'output_topic': LaunchConfiguration('output_topic'),
            'output_type': LaunchConfiguration('output_type'),
        }],
    )

    return LaunchDescription([
        input_topic_arg,
        input_type_arg,
        output_topic_arg,
        output_type_arg,
        encode_node,
        decode_node,
    ])
