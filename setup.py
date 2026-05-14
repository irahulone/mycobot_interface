from setuptools import find_packages, setup

package_name = 'mycobot_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
    # Required by ament to index the package
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    # Package manifest
    ('share/' + package_name, ['package.xml']),
    # Launch files
    ('share/' + package_name + '/launch', [
        'launch/joystick_control_vir.launch.py',
        'launch/joystick_control.launch.py',
        'launch/mycobot_280pi_vir.launch.py',
        'launch/mycobot_280pi.launch.py',  
    ]),
],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='my-cobot-2',
    maintainer_email='my-cobot-2@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'hard_control = mycobot_interface.hard_control:main',
            'soft_control = mycobot_interface.soft_control:main',
            'robot_joint_interface = mycobot_interface.robot_joint_interface:main',
            'robot_joint_interface_vir = mycobot_interface.robot_joint_interface_vir:main',
            'robot_cart_interface = mycobot_interface.robot_cart_interface:main',
            'goal_joint_state_pub = mycobot_interface.des_jstate_publisher:main',
            'goal_cart_state_pub = mycobot_interface.des_cstate_publisher:main',
            'ik_solver = mycobot_interface.ik_solver:main',
        ],
    },
)