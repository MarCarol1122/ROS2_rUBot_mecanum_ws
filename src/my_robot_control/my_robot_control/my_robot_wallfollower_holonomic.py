import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

# TF2 para giro preciso de 90°
import tf_transformations
from tf2_ros import Buffer, TransformListener

class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower_node')

        # Parameters
        self.declare_parameter('distance_limit', 0.5)    # desired distance to right wall
        self.declare_parameter('forward_speed', 0.20)    # linear speed
        self.declare_parameter('turn_speed', 0.40)       # angular speed
        self.declare_parameter('time_to_stop', 30.0)     # auto-stop
        self.declare_parameter('tolerance', 0.05)        # band around base_distance (RIGHT)

        self.base_distance = float(self.get_parameter('distance_limit').value)
        self.v_lin = float(self.get_parameter('forward_speed').value)
        self.v_ang = float(self.get_parameter('turn_speed').value)
        self.time_to_stop = float(self.get_parameter('time_to_stop').value)
        self.tol = float(self.get_parameter('tolerance').value)

        # Escape lateral antes de girar 90°
        self.escape_duration = 1.0
        self.front_escape_start = None

        # Variables para giro preciso
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.turning_90 = False
        self.turn_target_yaw = None

        # Last commanded twist (will be published periodically)
        self.cmd = Twist()

        self.prev_vx = 0.0
        self.prev_vy = 0.0
        self.front_wall_type = None
        
        # ROS 2 entities
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, qos_profile_sensor_data
        )
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timers
        self.info_timer = self.create_timer(1.0, self.log_info)
        self.stop_timer = self.create_timer(0.05, self.stop_watchdog)
        self.cmd_timer = self.create_timer(0.1, self.cmd_publish_timer_cb)

        self._state_action = "Idle"
        self._last_action_logged = None
        self._shutting_down = False

        self.start_time_s = self.get_clock().now().nanoseconds * 1e-9

        self.get_logger().info(
            "WallFollower with PRECISE 90° TURN + RIGHT WALL ALIGNMENT"
        )

    # ----------------------------------------------------------
    # Odom helpers
    # ----------------------------------------------------------
    def get_yaw(self):
        """Return yaw from /odom→base_link TF."""
        try:
            trans = self.tf_buffer.lookup_transform(
                'odom', 'base_link', rclpy.time.Time()
            )
            q = trans.transform.rotation
            _, _, yaw = tf_transformations.euler_from_quaternion(
                [q.x, q.y, q.z, q.w]
            )
            return yaw
        except:
            return None

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    # ----------------------------------------------------------
    def stop_watchdog(self):
        """Stop the robot after time_to_stop seconds."""
        if self._shutting_down:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.start_time_s >= self.time_to_stop:
            self.get_logger().info("Stopping due to timeout.")
            self.stop()

    # ----------------------------------------------------------
    def stop(self):
        self._shutting_down = True
        self.cmd = Twist()
        try:
            self.publisher.publish(self.cmd)
        except Exception:
            pass
        for t in [self.info_timer, self.stop_timer, self.cmd_timer]:
            try:
                t.cancel()
            except Exception:
                pass

    # ----------------------------------------------------------
    def cmd_publish_timer_cb(self):
        if self._shutting_down:
            return
        try:
            self.publisher.publish(self.cmd)
        except Exception:
            pass

    # ----------------------------------------------------------
    def laser_callback(self, scan):
        if self._shutting_down:
            return

        angle_min = math.degrees(scan.angle_min)
        angle_inc = math.degrees(scan.angle_increment)

        FRONT, FR_RIGHT, RIGHT, BACK_RIGHT, BACK = [], [], [], [], []

        for i, d in enumerate(scan.ranges):
            if not math.isfinite(d):
                continue
            if d < scan.range_min or d > scan.range_max:
                continue

            ang = angle_min + i * angle_inc

            if -20 <= ang <= 20:
                FRONT.append(d)
            elif -70 <= ang < -20:
                FR_RIGHT.append(d)
            elif -110 <= ang < -70:
                RIGHT.append(d)
            elif -160 <= ang < -110:
                BACK_RIGHT.append(d)
            elif -200 <= ang < -160:
                BACK.append(d)

        min_front = min(FRONT) if FRONT else float('inf')
        min_fr_right = min(FR_RIGHT) if FR_RIGHT else float('inf')
        min_right = min(RIGHT) if RIGHT else float('inf')
        min_back_right = min(BACK_RIGHT) if BACK_RIGHT else float('inf')
        min_back = min(BACK) if BACK else float('inf')

        twist = Twist()
        action = ""

        # -------------------------------------------
        # RESET si ya no hay obstáculo delante
        # -------------------------------------------
        if min_front >= self.base_distance:
            self.front_escape_start = None
            self.turning_90 = False
            self.turn_target_yaw = None

        # ==========================================================
        # RULE 1: OBSTÁCULO DELANTE → ESCAPE LATERAL + GIRO 90°
        # ==========================================================
        if min_front < self.base_distance:

            now = self.get_clock().now().nanoseconds * 1e-9

            # Girando 90° exacto
            if self.turning_90 and self.turn_target_yaw is not None:
                twist.linear.x = 0.0
                twist.linear.y = 0.0
                current_yaw = self.get_yaw()
                if current_yaw is None:
                    twist.angular.z = 0.2
                    action = "TURNING 90° (NO ODOM)"
                    self.cmd = twist
                    return
                error = self.normalize_angle(self.turn_target_yaw - current_yaw)
                if abs(error) > math.radians(2):
                    twist.angular.z = 0.4 * (error / abs(error))
                    action = f"TURNING 90° EXACT (error={math.degrees(error):.1f}°)"
                else:
                    self.turning_90 = False
                    self.turn_target_yaw = None
                    twist.angular.z = 0.0
                    action = "TURN COMPLETED → FORWARD"
                self.cmd = twist
                return

            # Escape lateral
            if self.front_escape_start is None:
                self.front_escape_start = now
            if now - self.front_escape_start < self.escape_duration:
                twist.linear.x = 0.0
                twist.linear.y = self.v_lin
                twist.angular.z = 0.0
                action = "FRONT → STRAFE LEFT (escape phase)"
            else:
                current_yaw = self.get_yaw()
                if current_yaw is not None:
                    self.turn_target_yaw = self.normalize_angle(current_yaw + math.radians(90))
                    self.turning_90 = True
                    action = "FRONT → STARTING PRECISE 90° TURN"
                else:
                    twist.angular.z = self.v_ang
                    action = "FRONT → TURNING (NO ODOM)"
            self.cmd = twist
            return

        # ==========================================================
        # RULE 2: CORRECCIÓN DE INCLINACIÓN RESPECTO A LA PARED
        # ==========================================================
        inclination_threshold = 0.05  # diferencia mínima para considerar inclinación
        if math.isfinite(min_fr_right) and math.isfinite(min_back_right):
            if min_fr_right > min_back_right + inclination_threshold:
                # cabeza más lejos → girar ligeramente hacia la pared
                twist.linear.x = self.v_lin * 0.5
                twist.linear.y = 0.0
                twist.angular.z = -self.v_ang * 0.6
                action = "ALIGN → nose drifting AWAY → rotate RIGHT"
                self.cmd = twist
                return
            elif min_back_right > min_fr_right + inclination_threshold:
                # cola más lejos → girar ligeramente hacia afuera
                twist.linear.x = self.v_lin * 0.5
                twist.linear.y = 0.0
                twist.angular.z = self.v_ang * 0.6
                action = "ALIGN → tail drifting AWAY → rotate LEFT"
                self.cmd = twist
                return

        # ==========================================================
        # RULE 3: FRONT-RIGHT obstacle → slow + left
        # ==========================================================
        if min_fr_right < self.base_distance:
            twist.linear.x = self.v_lin
            twist.linear.y = self.v_lin
            action = f"FRONT-RIGHT {min_fr_right:.2f} → DIAGONAL"

        # ==========================================================
        # RULE 4: RIGHT visible → tolerance band
        # ==========================================================
        elif math.isfinite(min_right):
            error = min_right - self.base_distance
            if abs(error) <= self.tol:
                twist.linear.x = self.v_lin
                twist.linear.y = 0.0
                action = "RIGHT OK → STRAIGHT"
            elif error < 0:
                twist.linear.x = self.v_lin * 0.5
                twist.linear.y = self.v_lin * 0.5
                action = "RIGHT too CLOSE → LEFT"
            else:
                twist.linear.x = self.v_lin * 0.5
                twist.linear.y = -self.v_lin * 0.5
                action = "RIGHT too FAR → RIGHT"

        # ==========================================================
        # RULE 5: BACK-RIGHT
        # ==========================================================
        elif math.isfinite(min_back_right) and (
            not math.isfinite(min_right) or min_back_right <= min_right
        ):
            twist.linear.x = self.v_lin
            twist.linear.y = -self.v_lin
            action = "BACK-RIGHT → DIAGONAL"

        else:
            action = "NO WALL → STOP"

        self.cmd = twist
        if action != self._last_action_logged:
            self.get_logger().info(action)
            self._last_action_logged = action
        self._state_action = action
        self.prev_vx = twist.linear.x
        self.prev_vy = twist.linear.y

    # ----------------------------------------------------------
    def log_info(self):
        if not self._shutting_down:
            self.get_logger().info(self._state_action)


def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop()
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

