#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import tf_transformations

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_control_node')
        
        # Robot state
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_f = 0.0
        self.closest_distance = float('inf')

        # Time start
        self.start_time = self.get_clock().now().seconds_nanoseconds()[0]

        # Subscribe to LIDAR
        self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )

        # Subscribe to odometry
        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Publisher for velocity commands
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Parameters
        self.declare_parameter('vx', 0.3)
        self.declare_parameter('vy', 0.0)
        self.declare_parameter('w', 0.0)
        self.declare_parameter('td', 3.0)
        
        self.vx = self.get_parameter('vx').value
        self.vy = self.get_parameter('vy').value
        self.w = self.get_parameter('w').value
        self.td = self.get_parameter('td').value
        
        # Timer
        self.timer = self.create_timer(0.1, self.move_robot)

    def lidar_callback(self, msg: LaserScan):
        """Reads LiDAR and stores the minimum distance."""
        filtered_ranges = [r for r in msg.ranges if r > 0.01]  # Remove invalid zeros
        if len(filtered_ranges) > 0:
            self.closest_distance = min(filtered_ranges)
        else:
            self.closest_distance = float('inf')

    def odom_callback(self, msg):
        """Updates robot position from odometry."""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.robot_f = tf_transformations.euler_from_quaternion(
            [q.x, q.y, q.z, q.w]
        )
    
    def move_robot(self):
        """Publishes velocity commands until a time limit or obstacle stops the robot."""
        elapsed_time = self.get_clock().now().seconds_nanoseconds()[0] - self.start_time
        
        vel = Twist()

        if elapsed_time < self.td:

            # STOP if obstacle closer than 30 cm
            if self.closest_distance < 0.3:
                self.get_logger().warn(f'Obstacle detected at {self.closest_distance:.2f} m — STOPPING!')
                vel.linear.x = 0.0
                vel.angular.z = 0.0
            else:
                # Normal movement
                self.get_logger().info(f'Moving... obstacle at {self.closest_distance:.2f} m')
                vel.linear.x = self.vx
                vel.linear.y = self.vy
                vel.angular.z = self.w
            
            self.publisher.publish(vel)

        else:
            # Time completed → stop
            self.get_logger().warn('Time limit reached — STOPPING ROBOT')
            self.publisher.publish(Twist())
            self.timer.cancel()
            rclpy.shutdown()

def main():
    rclpy.init()
    controller = RobotController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()

if __name__ == '__main__':
    main()
