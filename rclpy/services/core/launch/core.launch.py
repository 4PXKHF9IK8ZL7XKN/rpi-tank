#!/usr/bin/env python3

import signal
import sys
import time
 
import launch

from launch import LaunchDescription
import launch_ros.actions

"""Launch the cpp_code executable in this package"""
def generate_launch_description():
    ls = launch.LaunchDescription()
    return LaunchDescription([
        # the name of the executable is set in CMakeLists.txt, towards the end of
        # the file, in add_executable(...) and the directives following it
        launch_ros.actions.Node(package='rpi_gpio_tank', node_executable='core', output='screen'),
        launch_ros.actions.Node(package='image_tools', node_executable='cam2image',output='screen'),
    ])


if __name__ == '__main__':
    # ls = LaunchService(argv=argv, debug=True)  # Use this instead to get more debug messages.
    ls = launch.LaunchService(argv=sys.argv[1:])
    ls.include_launch_description(generate_launch_description())
    sys.exit(ls.run())
