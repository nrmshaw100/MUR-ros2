from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="joy",
            executable="game_controller_node",
            name="game_controller_node",
            parameters=[{"coalesce_interval_ms": 20}],
        ),
        Node(
            package="ground_station",
            executable="spi_monitor",
            name="ground_station_gui",
        ),
    ])
