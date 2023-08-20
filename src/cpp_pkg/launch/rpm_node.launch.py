from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        Node(
          package="cpp_pkg",
          executable="rpm_pub",
          name="rpm_pub_node",
          parameters=[
              { "rpm_value": 5.0 }
          ]
        ),
        ExecuteProcess(
          cmd=['ros2', 'topic', 'list'],
          output='screen'
        )
    ])