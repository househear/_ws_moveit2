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
from launch import LaunchDescription
import yaml
import xacro
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from webots_ros2_core.webots_launcher import WebotsLauncher
from webots_ros2_core.utils import ControllerLauncher
from ament_index_python.packages import get_package_share_directory
from webots_ros2_core.utils import ControllerLauncher
from webots_ros2_core.webots_launcher import WebotsLauncher
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource



def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None



def generate_launch_description():
    # Webots
   
    run_process_controller = Node(
        package='rock_rhino_process_controller',
        executable='rock_rhino_process_controller',
        name='rock_rhino_process_controller'
    )




    return launch.LaunchDescription([
        run_process_controller
    ])
