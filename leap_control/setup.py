from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'leap_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'src'), glob('src/*.py')),
        (os.path.join('share', package_name, 'cfg'), glob('cfg/*.yaml')),
        #('share/' + package_name, ['launch/fingers_nodes.py'])
    ],
    install_requires=['setuptools','dynamixel_sdk'],
    zip_safe=True,
    maintainer='beatrix',
    maintainer_email='marques.beatriz14@ua.pt',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'read_position = leap_control.individual_nodes.read_position:main',
            'read_velocity = leap_control.individual_nodes.read_velocity:main',
            'read_pwm = leap_control.individual_nodes.read_pwm:main',
            'manager = leap_control.multiple_readings_one_motor.manager_node:main',
            'read_position_from_manager_node = leap_control.multiple_readings_one_motor.read_position_from_manager_node:main',
            'read_velocity_from_manager_node = leap_control.multiple_readings_one_motor.read_velocity_from_manager_node:main',
            'read_current_from_manager_node = leap_control.multiple_readings_one_motor.read_current_from_manager_node:main',
            'set_position_one_motor = leap_control.multiple_readings_one_motor.set_position_one_motor:main',
            'finger_positions = leap_control.one_finger_control.read_positions_node:main',
            'finger_velocities = leap_control.one_finger_control.read_velocities_node:main',
            'finger_currents = leap_control.one_finger_control.read_currents_node:main',
            'set_finger_position = leap_control.one_finger_control.set_positions:main',
            'finger_grasping = leap_control.one_finger_control.grasping_node:main',
            'hand_manager = leap_control.hand_control.manager_node:main',
            'set_fingers_position = leap_control.hand_control.set_positions:main',
            'pick_up_demo = leap_control.demos.pick_up_demo:main',
            'finger_collision_demo = leap_control.demos.finger_collision_demo:main',
            'middle_manager = leap_control.hand_control.middle_node:main',
            'finger_manager = leap_control.hand_control.finger_node:main',
            'thumb_manager = leap_control.hand_control.thumb_node:main',
            'index_manager = leap_control.hand_control.index_node:main',
            'ring_manager = leap_control.hand_control.ring_node:main',
            'save_data = leap_control.hand_control.save_data_node:main',
            'check_colisions = leap_control.hand_control.check_colisions:main',
            'read_sensors = leap_control.fsr_sensors.fsr_sensors_node:main',
            'collect_data = leap_control.hand_control.collect_data:main',
            'leap_control_driver = src.leap_control_driver:main',
            'test = src.test:main',
        ],
    },
)
