from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='game_controller_node',
            name = 'gs_joy',
            arguments=['--ros-args', '-p', 'coalesce_interval_ms:=10']
        ),
        Node(
            package='jetson',
            executable='dualsense',
            name='gs_dualsense',
        ),
        Node(
            package='ground_station',
            executable='monitor_poller',
            name='gs_poller'
       ),
       Node(
            package='ground_station',
            executable='spi_monitor',
            name='gs_monitor'
       )
    ])