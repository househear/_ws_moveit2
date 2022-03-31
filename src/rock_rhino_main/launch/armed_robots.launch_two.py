#!/usr/bin/env python

# Copyright 1996-2021 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch Webots and the controllers."""

import os
import launch
from ament_index_python.packages import get_package_share_directory
from webots_ros2_core.utils import ControllerLauncher
from webots_ros2_core.webots_launcher import WebotsLauncher


def generate_launch_description():
    # Webots
    webots = WebotsLauncher(world=os.path.join(get_package_share_directory('rock_rhino_main'), 'worlds', 'armed_robots.wbt'))

    # Controller nodes
    synchronization = launch.substitutions.LaunchConfiguration('synchronization', default=False)
    abb_controller = ControllerLauncher(
        package='webots_ros2_core',
        executable='webots_robotic_arm_node',
        arguments=['--webots-robot-name=abbirb4600', '--webots-node-name=webots_interface_abb'],
        parameters=[{'synchronization': synchronization, 'use_joint_state_publisher': False, 'controller_name': 'abb_arm_controller'}],
        output='screen'
    )
    ure5_controller = ControllerLauncher(
        package='webots_ros2_core',
        executable='webots_robotic_arm_node',
        arguments=['--webots-robot-name=UR5e', '--webots-node-name=webots_interface_ur'],
        parameters=[{'synchronization': synchronization, 'use_joint_state_publisher': False, 'controller_name': 'ur_arm_controller', }],
        output='screen'
    )

    # Control nodes
    armed_robots_ur = ControllerLauncher(
         package='rock_rhino_main',
         executable='armed_robots_ur',
         name = 'webots_ur_client',
         output='screen'
     )
    armed_robots_abb = ControllerLauncher(
        package='rock_rhino_main',
        executable='armed_robots_abb',
        name = 'webots_abb_client',
        output='screen'
    )
    return launch.LaunchDescription([
        webots, abb_controller, ure5_controller, 
        armed_robots_abb, armed_robots_ur,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        ),
    ])
