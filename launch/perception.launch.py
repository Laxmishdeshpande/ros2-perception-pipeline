from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        
        Node(
            package='day24_perception',
            executable='camera_publisher',
            name='camera_publisher',
            output='screen'
        ),
        
        Node(
            package='day24_perception',
            executable='vo_node',
            name='vo_node',
            output='screen'
        ),

        Node(
            package='day24_perception',
            executable='yolo_node',
            name='yolo_node',
            output='screen'
        ),

        Node(
            package='day24_perception',
            executable='seg_node',
            name='seg_node',
            output='screen'
        ),

        Node(
            package='day24_perception',
            executable='navigation_node',
            name='navigation_node',
            output='screen'
        ),

        Node(
            package='day24_perception',
            executable='tf2_broadcaster',
            name='tf2_broadcaster',
            output='screen'
        ),
        
    ])