# Copyright 2018 Open Source Robotics Foundation, Inc.
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

"""Launch a add_two_ints_server and a (synchronous) add_two_ints_client."""

import launch
from launch_ros.actions import Node

def generate_launch_description():


    
    mongodb_server_node = Node(
        package="warehouse_ros_mongo",
        executable="mongo_wrapper_ros.py",
        parameters=[
            {"warehouse_port": 33829},
            {"warehouse_host": "localhost"},
            {"warehouse_plugin": "warehouse_ros_mongo::MongoDatabaseConnection"},
        ],
        output="screen",
    )
    
    db_server_node = Node(
    package="db_test",
    executable="db_action_server",
    #name="db_action_server",
    )
    return launch.LaunchDescription([
        mongodb_server_node,
        db_server_node,
        # TODO(wjwwood): replace this with a `required=True|False` option on ExecuteProcess().
        # Shutdown launch when client exits.
        #launch.actions.RegisterEventHandler(
        #    event_handler=launch.event_handlers.OnProcessExit(
        #        target_action=client,
        #        on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
        #    )),
    ])
