from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    moveit_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                FindPackageShare("moveit2_tutorials").find("moveit2_tutorials"),
                "launch/demo.launch.py"
            )
        )
    )

    planner_node = Node(
        package="panda_planner",
        executable="planner_node",
        output="screen"
    )

    return LaunchDescription([
        moveit_demo,
        planner_node
    ])
