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


import launch_ros.actions
def generate_launch_description():
    # Webots
    webots = WebotsLauncher(world=os.path.join(get_package_share_directory('rock_rhino_main'), 'worlds', 'armed_robots.wbt'))
    
    
    launch_include_tcpip = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(
            get_package_share_directory('rock_rhino_tcpip_service'),
            'launch/services/add_two_ints_async.launch.py'))
    )


    launch_include_db = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(
            get_package_share_directory('db_test'),
            'launch/db_server.launch.py'))
    )



    launch_include_image_processor = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(
            get_package_share_directory('rock_rhino_image_processor'),
            'launch/_ver01_image_processor.py'))
    )   

    package_dir = get_package_share_directory('webots_ros2_universal_robot')


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
        parameters=[{'synchronization': synchronization, 'use_joint_state_publisher': False, 'controller_name': 'ur_arm_controller'}],
        output='screen'
    )

    device_config_panda = {
    'camera_panda': {'topic_name': 'camera1', 'timestep': 16}
    }

    panda_controller = ControllerLauncher(
        package='webots_ros2_core',
        executable='webots_robotic_arm_node',
        arguments=['--webots-robot-name=Panda', '--webots-node-name=webots_interface_panda'],
        parameters=[{'synchronization': synchronization, 
                     'use_joint_state_publisher': False, 
                     'controller_name': 'panda_arm_controller',
                     'device_config': ''}],
        output='screen'
    )


    tag_controller = ControllerLauncher(
        package='webots_ros2_tutorials',
        executable='robot_enable',
        arguments=['--webots-robot-name=tag_robot', '--webots-node-name=webots_interface_tag'],
        parameters=[{'synchronization': synchronization, 'use_joint_state_publisher': False, 'controller_name': 'tag_robot_controller'}],
        output='screen'
    )

     #Control nodes
    armed_robots_ur = ControllerLauncher(
         package='rock_rhino_main',
         executable='armed_robots_ur',
         output='screen'
    )
    armed_robots_abb = ControllerLauncher(
        package='rock_rhino_main',
        executable='armed_robots_abb',
        output='screen'
    
    )

    image_commander = Node(
        package='rock_rhino_image_processor',
        executable='aruco_controller',
        name='aruco_controller'
    )

    run_process_controller = ControllerLauncher(
        package='rock_rhino_process_controller',
        executable='rock_rhino_process_controller',
        name='rock_rhino_process_controller',
        output='screen'
    )




    # planning_context
    robot_description_config = xacro.process_file(
        os.path.join(
            get_package_share_directory("moveit_resources_panda_moveit_config"),
            "config",
            "panda.urdf.xacro",
        )
    )
    robot_description = {"robot_description": robot_description_config.toxml()}

    robot_description_semantic_config = load_file(
        "moveit_resources_panda_moveit_config", "config/panda.srdf"
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_config
    }

    kinematics_yaml = load_yaml(
        "moveit_resources_panda_moveit_config", "config/kinematics.yaml"
    )
    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}

    # Planning Functionality
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_yaml = load_yaml(
        "moveit_resources_panda_moveit_config", "config/ompl_planning.yaml"
    )
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    # Trajectory Execution Functionality
    moveit_simple_controllers_yaml = load_yaml(
        "moveit_resources_panda_moveit_config", "config/panda_controllers.yaml"
    )
    moveit_controllers = {
        "moveit_simple_controller_manager": moveit_simple_controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
        ],
    )



    # MoveGroupInterface demo executable
    run_move_group_demo = Node(
        name="rock_rhino_moveit2",
        package="rock_rhino_moveit2",
        executable="run_ompl_constrained_planning_while_run",
        output="screen",
        #    prefix='kitty -e gdb -e run --args',
        parameters=[robot_description, robot_description_semantic, kinematics_yaml],
    )





    # RViz
    rviz_config_file = (
        get_package_share_directory("run_ompl_constrained_planning_")
        + "/launch/run_move_group.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_yaml,
        ],
    )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "panda_link0"],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("moveit_resources_panda_moveit_config"),
        "config",
        "panda_ros_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, ros2_controllers_path],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )

        # Load controllers
    load_controllers = []
    for controller in ["panda_arm_controller", "joint_state_controller"]:
        load_controllers += [
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner.py {}".format(controller)],
                shell=True,
                output="screen",
            )
        ]


    # Warehouse mongodb server
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
    name="db_action_server",
    )


    return launch.LaunchDescription([
        webots, 
        launch_include_tcpip,
        panda_controller,
        image_commander,
        rviz_node,
        static_tf,
        robot_state_publisher,
        run_move_group_node,
        run_move_group_demo,
        run_process_controller,
        ros2_control_node,
        mongodb_server_node,
        db_server_node,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        ),
    ]
    + load_controllers

    )
