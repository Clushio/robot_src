from launch import LaunchDescription
from launch_ros.actions import Node


def static_transform(name, x, parent, child):
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=name,
        arguments=[
            "--x",
            str(x),
            "--y",
            "0",
            "--z",
            "0",
            "--roll",
            "0",
            "--pitch",
            "0",
            "--yaw",
            "0",
            "--frame-id",
            parent,
            "--child-frame-id",
            child,
        ],
        output="screen",
    )


def generate_launch_description():
    return LaunchDescription(
        [
            static_transform("base_tf", -0.33, "body", "base_link"),
            static_transform("livox_tf", 0.33, "base_link", "livox_frame"),
        ]
    )
