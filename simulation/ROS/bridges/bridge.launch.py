from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os


def generate_launch_description():
    world_file = os.path.join(
        "/home/ros-gazebo/drone_ws/src/martian_drone/worlds", "FinalDrone22.sdf"
    )


    return LaunchDescription([

        # Start Gazebo with the SDF world
        ExecuteProcess(
            cmd=["ign", "gazebo", world_file, "-v", "4"],
            output="screen"
        ),

        # Bridge for Depth Camera (PointCloud2)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/depth_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked'
            ],
            output='screen'
        ),
        
        # Bridge for IMU sensor (use actual topic name from Gazebo)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU'
            ],
            output='screen'
        ),
        
        # Bridge for RGB Camera (Image data)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/rgb_camera@sensor_msgs/msg/Image@gz.msgs.Image'
            ],
            output='screen'
        ),

        # Bridge for Velocity Command (cmd_vel)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/X3/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'
            ],
            output='screen'
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '-1.57', '0', 'map', 'X3/depth_camera_link/depth_camera'],
            output='screen'
        ),

        # Run the drone_control.py script
        Node(
            package='martian_drone',
            executable='drone_controll',
            output='screen'
        )
    ])
