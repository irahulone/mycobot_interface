import os
from ament_index_python import get_package_share_directory, get_package_share_path
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    res = []

    port_launch_arg = DeclareLaunchArgument(
        name="port",
        default_value="/dev/ttyUSB0"
    )
    res.append(port_launch_arg)

    baud_launch_arg = DeclareLaunchArgument(
        name="baud",
        default_value="1000000"
    )
    res.append(baud_launch_arg)

    model_launch_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(
            get_package_share_directory("mycobot_description"),
            "urdf/mycobot_280_pi/mycobot_280_pi.urdf"
        )
    )
    res.append(model_launch_arg)

    rvizconfig_launch_arg = DeclareLaunchArgument(
        name="rvizconfig",
        default_value=os.path.join(
            get_package_share_directory("mycobot_280pi"),
            "config/mycobot_pi.rviz"
        )
    )
    res.append(rvizconfig_launch_arg)

    gui_launch_arg = DeclareLaunchArgument(
        name="gui",
        default_value="false"
    )
    res.append(gui_launch_arg)

    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        name="robot_state_publisher",
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description': robot_description}]
    )
    res.append(robot_state_publisher_node)


    """
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "world", "g_base"],
        output="screen"
    )
    res.append(static_tf_node)
    """
    
    
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_path("mycobot_280_moveit2"),
                "launch",
                "move_group.launch.py"
            )
        ),
        launch_arguments={
            "use_sim_time": "true"
        }.items()
    )
    res.append(moveit_launch)
    
    
    """
    mycobot_iksolver_node = Node(
        name="ik_solver",
        package="mycobot_interface",
        executable="ik_solver",
        output="screen"
    )
    res.append(mycobot_iksolver_node)
    """
    
    
    mycobot_joint_interface_node = Node(
        name="robot_joint_interface",
        package="mycobot_interface",
        executable="robot_joint_interface",
        output="screen",
        parameters = [{
             "port": LaunchConfiguration("port"),
             "baud": LaunchConfiguration("baud")
        }]
    )
    res.append(mycobot_joint_interface_node)
    
    
    rviz_node = Node(
        name="rviz2",
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=['-d', LaunchConfiguration("rvizconfig")],
    )
    res.append(rviz_node)

    return LaunchDescription(res)


