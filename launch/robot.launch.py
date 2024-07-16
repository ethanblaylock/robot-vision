import os
 
from ament_index_python.packages import get_package_share_directory
 
 
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess
 
from launch_ros.actions import Node
from launch.substitutions import Command
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart


def generate_launch_description():
 
 
    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!
 
    package_name='robot-vision' 
 
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'false'}.items()
    )
 
    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])

    controller_params = os.path.join(
        get_package_share_directory('articubot_one'), # <-- Replace with your package name
        'config',
        'my_controllers.yaml'
        )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_description},
                controller_params],
        )
             
 
    delayed_controller_manager = TimerAction(period=3.0,actions=[controller_manager])

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["diff_cont"],
    )

    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_drive_spawner],
        )
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_broad"],
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner],
        )
    )
    message_transmit1 = Node(package='ros_gz_bridge', executable='parameter_bridge',
                        arguments=['/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'])

    message_transmit2 = Node(package='ros_gz_bridge', executable='parameter_bridge',
                        arguments=['/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V'])


    message_transmit3 = Node(package='ros_gz_bridge', executable='parameter_bridge',
                        arguments=['/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model'])

    message_transmit4 = Node(package='ros_gz_bridge', executable='parameter_bridge',
                        arguments=['/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry'])

 

    #ros2 run ros_gz_bridge parameter_bridge /cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist

    # Launch them all!
    return LaunchDescription([
        rsp,
        delayed_controller_manager,
        delayed_diff_drive_spawner,
        delayed_joint_broad_spawner,
        message_transmit1,
        message_transmit2,
        message_transmit3,
        message_transmit4,
    ])