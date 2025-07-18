from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    sensor_ip = '169.254.223.207'
    map_file  = '/home/hassan/map-000.ply'
    csv_file  = '/home/hassan/live_pose.csv'
    

    # CLI localization with multicast (no sensor reprogramming)
    # Launch ouster-cli localization in unicast mode
    ouster_cli_cmd = ExecuteProcess(
        cmd=['/usr/local/bin/ouster-cli', 'source', sensor_ip, 
             'localize', map_file, 
             'save_trajectory', csv_file],
        output='screen'
    )

    bridge_node = Node(
        package='ouster_bridge',
        executable='csv_tf_bridge',
        arguments=['--file', csv_file, '--publish_odom'],
        output='screen'
    )

    return LaunchDescription([
        ouster_cli_cmd, 
        bridge_node
    ])

