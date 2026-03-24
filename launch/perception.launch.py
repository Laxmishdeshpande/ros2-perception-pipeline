from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # WHY: this function is what ROS2 calls when you run the launch file
    # it returns a LaunchDescription containing every node to start
    
    return LaunchDescription([
        
        Node(
            package='day24_perception',      # WHY: which package contains this node
            executable='camera_publisher',    # WHY: the entry point name from setup.py
            name='camera_publisher',          # WHY: the name this node registers with ROS2
            output='screen'                   # WHY: print logs to terminal instead of log file
        ),
        
        Node(
            package='day24_perception',      # WHY: same package
            executable='navigation_node',     # WHY: second entry point from setup.py
            name='navigation_node',
            output='screen'                   # WHY: see both nodes printing in same terminal
        ),
        
    ])