# Copyright 1996-2020 Soft_illusion.
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

from glob import glob
from setuptools import setup

package_name = 'rock_rhino_image_processor'

data_files = []
data_files.append(('share/ament_index/resource_index/packages', [
    'resource/' + package_name
]))
data_files.append(('share/' + package_name, [
    'launch/line_following_launch.py',
    'launch/demo_detection_launch.py',
    'launch/_ver01_image_processor.py'
]))
data_files.append(('share/' + package_name + '/worlds', [
    'worlds/custom_line_follower.wbt',
    'worlds/custom_ar_tag.wbt'
]))
data_files.append(('share/' + package_name + '/protos', [
    'protos/RobotSense.proto',
    'protos/RobotSenseCamera.proto'
]))

data_files.append(
    ('share/' + package_name + '/protos/icons', glob('protos/icons/*')))
data_files.append(
    ('share/' + package_name + '/worlds/textures', glob('worlds/textures/*')))
data_files.append(
    ('share/' + package_name + '/protos/textures', glob('protos/textures/*')))

data_files.append(('share/' + package_name, [
    'package.xml'
]))

setup(
    name=package_name,
    version='1.0.6',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools', 'launch'],
    zip_safe=True,
    keywords=['ROS2', 'Webots', 'Soft_Illusion',
              'Tutorials', 'Youtube', 'Simulation'],
    maintainer='Soft_illusion',
    maintainer_email='harsh.b.kakashaniya@gmail.com',
    description='Projects for videos for webots ros2 tutorial series on youtube',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'enable_robot = rock_rhino_image_processor.slave:main',
            'line_follower = rock_rhino_image_processor.master:main',
            'robot_enable = rock_rhino_image_processor.robot_enable:main',
            'aruco_controller = rock_rhino_image_processor.aruco_controller:main',
            'demo_controller = rock_rhino_image_processor.demo_controller:main'
        ],
        'launch.frontend.launch_extension': ['launch_ros = launch_ros']
    }
)
