import os
 
from ament_index_python.packages import get_package_share_directory
 
 
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess
 
from launch_ros.actions import Node
 
 
 
def generate_launch_description():
 
 
    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!
 
    package_name='robot-vision' 
 
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )
 
    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = ExecuteProcess(cmd=['gz','sim','empty.sdf'])
             
 
    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', '/robot_description',
                                   '-entity', 'my_bot'],
                        output='screen')
 
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
        gazebo,
        spawn_entity,
        message_transmit1,
        message_transmit2,
        message_transmit3,
        message_transmit4,
    ])