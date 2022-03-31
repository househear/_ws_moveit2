"""webots_ros2 package setup file."""

from setuptools import setup

package_name = 'rock_rhino_main'
worlds = [
    'worlds/armed_robots.wbt',
    'worlds/.armed_robots.wbproj'
]
launchers = [
    'launch/armed_robots.launch.py'
]

data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name, launchers))
data_files.append(('share/' + package_name + '/worlds', worlds))
data_files.append(('share/' + package_name, ['package.xml']))
data_files.append(('share/' + package_name + '/protos', [
    'protos/Panda.proto',
    'protos/RobotSense.proto',
    'protos/RobotSenseCamera.proto'
]))

data_files.append(('share/' + package_name + '/controllers/_superman/build/release', [
    'controllers/_superman/build/release/_superman',
    'controllers/_superman/build/release/_superman.d',
    'controllers/_superman/build/release/_superman.o',
    'controllers/_superman/build/release/libopencv_core.so.4.5.2'
]))

data_files.append(('share/' + package_name + '/textures', [
    'textures/ar_tag.jpg'
]))

setup(
    name=package_name,
    version='1.0.6',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools', 'launch'],
    zip_safe=True,
    author='Cyberbotics',
    author_email='support@cyberbotics.com',
    maintainer='Cyberbotics',
    maintainer_email='support@cyberbotics.com',
    keywords=['ROS', 'Webots', 'Robot', 'Simulation', 'Demos'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Various demos of the Webots-ROS2 interface.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'armed_robots_ur = rock_rhino_main.armed_robots_ur:main',
            'armed_robots_abb = rock_rhino_main.armed_robots_abb:main'
        ],
        'launch.frontend.launch_extension': ['launch_ros = launch_ros']
    }
)
