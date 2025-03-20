## Launches all scripts

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Node(
        #     package='pkgname'
        #     executable='pythscript.py'
        #     name='shownname'
        # )
    ])