import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math


class RobotSelfControlHolonomic(Node):

    def __init__(self):
        super().__init__('robot_selfcontrol_holonomic')

        # Parameters
        self.declare_parameter('distance_limit', 0.4)
        self.declare_parameter('forward_speed', 0.2)
        self.declare_parameter('side_speed', 0.2)
        self.declare_parameter('time_to_stop', 10.0)

        self._distanceLimit = self.get_parameter('distance_limit').value
        self._forwardSpeed = self.get_parameter('forward_speed').value
        self._sideSpeed = self.get_parameter('side_speed').value
        self._timeToStop = self.get_parameter('time_to_stop').value

        # Message
        self._msg = Twist()

        # Publishers + timers
        self._cmdVel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.05, self.timer_callback)

        # Laser subscriber
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10
        )

        # Time counters
        self.start_time = self.get_clock().now().nanoseconds * 1e-9
        self._shutting_down = False

    # -------------------------------------------
    # TIMER → sends velocity to robot
    # -------------------------------------------
    def timer_callback(self):
        if self._shutting_down:
            return

        now = self.get_clock().now().nanoseconds * 1e-9
        elapsed = now - self.start_time

        # Publish current twist command
        self._cmdVel.publish(self._msg)

        # Stop after timeout
        if elapsed >= self._timeToStop:
            self.stop()
            self.get_logger().info("Time finished: Robot stopped")
            rclpy.try_shutdown()

    # -------------------------------------------
    # LASER CALLBACK
    # -------------------------------------------
    def laser_callback(self, scan):
        if self._shutting_down:
            return

        angle_min = scan.angle_min * 180.0 / math.pi
        angle_inc = scan.angle_increment * 180.0 / math.pi

        custom_range = []
        for i, d in enumerate(scan.ranges):
            if not math.isfinite(d):
                continue
            angle_deg = angle_min + i * angle_inc
            if angle_deg > 180:
                angle_deg -= 360
            if -150 < angle_deg < 150:
                custom_range.append((d, angle_deg))

        if not custom_range:
            return

        # Closest object
        closest_dist, angle = min(custom_range)

        # Determine zone
        if -45 <= angle <= 45:
            zone = "FRONT"
        elif 45 < angle <= 110:
            zone = "LEFT"
        elif -110 <= angle < -45:
            zone = "RIGHT"
        elif angle > 110:
            zone = "BACK_LEFT"
        elif angle < -110:
            zone = "BACK_RIGHT"
        else:
            zone = "UNKNOWN"

        # Debug info
        self.get_logger().info(
            f"[LASER] d={closest_dist:.2f}m angle={angle:.1f}° → {zone}"
        )

        # -------------------------------------------
        # HOLONOMIC BEHAVIOR
        # -------------------------------------------
        if closest_dist < self._distanceLimit:
            if zone == "FRONT":
                # Move backward holonomically
                self._msg.linear.x = -self._forwardSpeed
                self._msg.linear.y = 0.0
                self._msg.angular.z = 0.0

            elif zone == "RIGHT":
                # Move LEFT (positive y)
                self._msg.linear.x = 0.0
                self._msg.linear.y = +self._sideSpeed
                self._msg.angular.z = 0.0

            elif zone == "LEFT":
                # Move RIGHT (negative y)
                self._msg.linear.x = 0.0
                self._msg.linear.y = -self._sideSpeed
                self._msg.angular.z = 0.0

            elif zone == "BACK_LEFT":
                # Escape forward-right
                self._msg.linear.x = +self._forwardSpeed
                self._msg.linear.y = -self._sideSpeed

            elif zone == "BACK_RIGHT":
                # Escape forward-left
                self._msg.linear.x = +self._forwardSpeed
                self._msg.linear.y = +self._sideSpeed

        else:
            # No obstacles → go forward
            self._msg.linear.x = self._forwardSpeed
            self._msg.linear.y = 0.0
            self._msg.angular.z = 0.0

    # -------------------------------------------
    # STOP FUNCTION
    # -------------------------------------------
    def stop(self):
        self._shutting_down = True
        stop = Twist()
        self._cmdVel.publish(stop)

def main(args=None):
    rclpy.init(args=args)
    node = RobotSelfControlHolonomic()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
