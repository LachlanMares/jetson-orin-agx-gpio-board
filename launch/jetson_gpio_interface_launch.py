#! /usr/bin/env python3

import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='jetson_orin_agx_gpio_board',
            executable='jetson_gpio_interface.py',
            name='jetson_gpio_interface'),
  ])
