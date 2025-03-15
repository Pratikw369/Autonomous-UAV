import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, PointCloud2
import time
import sensor_msgs.point_cloud2 as pc2
import numpy as np


class DroneControl(Node):
    def __init__(self):
        super().__init__('drone_control')

        # Publisher (to control the drone)
        self.publisher_ = self.create_publisher(Twist, '/X3/cmd_vel', 10)

        # Subscribers (to receive sensor data)
        self.subscription_imu = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.subscription_depth = self.create_subscription(PointCloud2, '/depth_camera/points', self.depth_callback, 10)

        # Drone State Variables
        self.home_position = None
        self.current_altitude = 0.0
        self.reached_hover_height = False
        self.hover_timer_started = False
        self.landing_initiated = False
        self.hovering_time = 30  # Hover for at least 30 seconds
        self.min_hover_height = 3.0  # Minimum required height
        self.landing_threshold = 0.1  # Margin for verifying landing

        self.get_logger().info("Drone control node initialized")
        self.takeoff()

    def imu_callback(self, msg):
        """ Extract altitude from IMU orientation (Assuming Z-axis gives height) """
        self.current_altitude = msg.orientation.z  # Approximate height estimation
        self.get_logger().info(f"Current Altitude: {self.current_altitude:.2f} m")

        # Set home position once
        if self.home_position is None:
            self.home_position = self.current_altitude
            self.get_logger().info(f"Home position set: {self.home_position:.2f} m")

        # Stop ascending once we reach hover height
        if not self.reached_hover_height and self.current_altitude >= self.min_hover_height:
            self.reached_hover_height = True
            self.get_logger().info("Reached hover height! Maintaining hover...")
            self.stop_movement()  # Stop ascending
            self.start_hover_timer()

    def depth_callback(self, msg):
        """ Processes depth camera data to detect ground level """
        points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        np_points = np.array(points)

        if np_points.size > 0:
            min_z = np.min(np_points[:, 2])  # Find lowest point
            self.get_logger().info(f"Lowest detected point: {min_z:.2f} m")

    def takeoff(self):
        """ Gradually ascend until reaching 3m """
        self.get_logger().info("Initiating takeoff")
        twist = Twist()
        twist.linear.z = 1.0  # Constant ascent speed
        self.publisher_.publish(twist)

    def stop_movement(self):
        """ Stops drone movement (used for stopping ascent) """
        self.get_logger().info("Stopping movement")
        twist = Twist()
        twist.linear.z = 0.0
        self.publisher_.publish(twist)

    def start_hover_timer(self):
        """ Start hover timer (ensures at least 30s of hover) """
        if not self.hover_timer_started:
            self.hover_timer_started = True
            self.get_logger().info("Starting hover timer for 30 seconds...")
            self.create_timer(self.hovering_time, self.initiate_landing)

    def initiate_landing(self):
        """ Start controlled descent after hovering """
        if not self.landing_initiated:
            self.get_logger().info("Initiating landing...")
            self.landing_initiated = True
            twist = Twist()
            twist.linear.z = -0.5  # Controlled descent
            self.publisher_.publish(twist)
            self.create_timer(5.0, self.land)

    def land(self):
        """ Stop drone movement at home position """
        self.get_logger().info("Checking final landing position...")

        # Check if the final altitude matches the home position within threshold
        if abs(self.current_altitude - self.home_position) <= self.landing_threshold:
            self.get_logger().info(f"✅ Successfully landed at home position (Altitude: {self.current_altitude:.2f} m)")
        else:
            self.get_logger().warn(f"❌ Landing offset detected! Final altitude: {self.current_altitude:.2f} m, "
                                   f"Expected: {self.home_position:.2f} m")

        self.stop_movement()


def main(args=None):
    rclpy.init(args=args)
    drone_control = DroneControl()
    rclpy.spin(drone_control)
    drone_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
