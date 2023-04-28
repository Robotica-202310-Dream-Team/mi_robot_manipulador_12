from setuptools import setup

package_name = 'mi_robot_manipulador_12'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='danielvillar',
    maintainer_email='danielvillar@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_manipulator_teleop = mi_robot_manipulador_12.robot_manipulator_teleop:main',
            'SerialRaspESP = mi_robot_manipulador_12.SerialRaspESP:main',
            'robot_manipulator_planner = mi_robot_manipulador_12.robot_manipulator_planner:main',
            'robot_manipulator_interface = mi_robot_manipulador_12.robot_manipulator_interface:main',
            'robot_manipulator_directK = mi_robot_manipulador_12.robot_manipulator_directK:main',
            'Serial_writer_manipulator = mi_robot_manipulador_12.Serial_writer_manipulator:main',
            'robot_manipulator_joystick_teleop = mi_robot_manipulador_12.robot_manipulator_joystick_teleop:main',
        ],  
    },
)
