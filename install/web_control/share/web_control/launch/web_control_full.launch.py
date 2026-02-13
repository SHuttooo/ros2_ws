import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    return LaunchDescription([
        # 1. Rosbridge WebSocket (Port 9090)
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen'
        ),
        
        # 2. Web Video Server (Port 8080)
        Node(
            package='web_video_server',
            executable='web_video_server',
            name='web_video_server',
            output='screen',
            parameters=[{'port': 8080}]
        ),
        
        # 3. Backend Custom (Logique + Serveur Web Port 8000)
        Node(
            package='web_control',
            executable='backend_node',
            name='backend_node',
            output='screen'
        )
    ])
