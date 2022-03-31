from setuptools import setup

package_name = 'rock_rhino_process_controller'
launchers = [
    'launch/armed_robots.launch.py'
]
setup(
    name=package_name,
    version='2.3.2',
    packages=[],
    py_modules=[
        'rock_rhino_process_controller'
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, launchers),
    ],
    
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Chris Lalancette',
    maintainer_email='clalancette@openrobotics.org',
    author='Graylin Trevor Jay, Austin Hendrix',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: BSD',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='A robot-agnostic teleoperation node to convert keyboard'
                'commands to Twist messages.',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rock_rhino_process_controller = rock_rhino_process_controller:main'
        ],
         'launch.frontend.launch_extension': ['launch_ros = launch_ros']
    },
)
