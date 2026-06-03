#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32, Bool
import numpy as np
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as R

class LineFollowerController(Node):
    """
    Point-to-point proportional controller with traffic light awareness.
    """

    def __init__(self):
        super().__init__('line_follower_controller')

        self.declare_parameter('linear_speed', 0.15)
        self.declare_parameter('curve_speed', 0.07)
        self.declare_parameter('curve_threshold', 50.0)

        self.declare_parameter('kp_straight', 0.0015)
        self.declare_parameter('kp_curve', 0.0040)
        self.declare_parameter('kd', 0.0030)

        self.declare_parameter('max_angular_vel', 1.5)
        self.declare_parameter('alpha', 0.5)

        self.linear_speed = self.get_parameter('linear_speed').value
        self.curve_speed = self.get_parameter('curve_speed').value
        self.curve_threshold = self.get_parameter('curve_threshold').value

        self.kp_straight = self.get_parameter('kp_straight').value
        self.kp_curve = self.get_parameter('kp_curve').value
        self.kd = self.get_parameter('kd').value

        self.w_max = self.get_parameter('max_angular_vel').value
        self.alpha = self.get_parameter('alpha').value

        # State
        self.line_error = 0.0
        self.prev_error = 0.0
        self.tl_state   = 'UNKNOWN'
        self.sign_state = "NONE"
        self.last_motion_state = ""
        self.current_linear_vel = 0.0
        self.acceleration = 0.01   # rampa de velocidad
        self.curve_state = False
        self.line_lost_threshold = 140
        self.filtered_error = 0.0
        self.intersection_detected = False
        
        #Odometry
        self.current_heading = 0.0
        self.start_heading = 0.0
        self.target_angle = np.pi / 2
        self.current_x = 0.0
        self.current_y = 0.0
        self.start_x = 0.0
        self.start_y = 0.0
        self.target_distance = 0.5


        # -----------------------------------------
        # FSM
        # -----------------------------------------
        self.mode = "FOLLOW" # FOLLOW # MANEUVER # SPECIAL

        # velocidad de maniobra
        self.turn_linear_speed = 0.05
        self.turn_angular_speed = 1.0

        self.pending_action = "NONE"
        self.special_behavior = "NONE"
        self.action_start_time = None
        self.turn_duration = 2.0
        self.special_start_time = None
        self.special_duration = 5.0

        # ROS I/O
        self.pub_vel  = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub_line = self.create_subscription(Float32,'/line_error',self._line_callback,1)
        self.sub_tl   = self.create_subscription(String, '/traffic_light/state', self._tl_callback, 10)
        self.sub_sign = self.create_subscription(String, '/traffic_sign/state', self._sign_callback,10)
        self.sub_odom = self.create_subscription(Odometry,'/odom',self._odom_callback,10)
        self.sub_fsm = self.create_subscription(String,'/fsm_command',self._fsm_command_callback,10)
        self.sub_intersection = self.create_subscription(Bool,'/intersection_detected',self._intersection_callback,10)

        # Control loop at 20 Hz
        self.timer = self.create_timer(0.05, self._control_loop)
        self.get_logger().info('LineFollowerController Started')

    # Callbacks
    def _line_callback(self, msg: Float32):
        self.filtered_error = (self.alpha * self.filtered_error +(1 - self.alpha) * msg.data)
        self.line_error = self.filtered_error

    def _tl_callback(self, msg: String):
        self.tl_state = msg.data

    def _intersection_callback(self, msg):
        self.intersection_detected = msg.data

    def _odom_callback(self, msg):
        q = msg.pose.pose.orientation
        quat = [
            q.x,
            q.y,
            q.z,
            q.w
        ]
        yaw = R.from_quat(quat).as_euler(
            'xyz'
        )[2]
        self.current_heading = yaw
        self.current_x = (msg.pose.pose.position.x)
        self.current_y = (msg.pose.pose.position.y)

    def _fsm_command_callback(self, msg):
        command = msg.data
        if (self.mode == "STOPPED" and command == "START"):
            self.pending_action = "NONE"
            self.mode = "FOLLOW"
            self.get_logger().info("Exiting STOPPED mode")

    def _sign_callback(self, msg):
        self.sign_state = msg.data

        # guardar acción pendiente
        if self.sign_state in [
            "LEFT",
            "RIGHT",
            "STRAIGHT",
            "STOP"]:
            self.pending_action = self.sign_state

        elif self.sign_state in [
            "ROUND",
            "GIVE_WAY",
            "WORKERS"]:

            self.special_behavior = self.sign_state

    def update_fsm(self):
        #STOP
        if self.pending_action == "STOP" and self.mode != "STOPPED":
            self.mode = "STOPPED"
            self.get_logger().info("Entering STOPPED mode")
            return
        # FOLLOW -> MANEURVER
        if (self.mode == "FOLLOW" and self.tl_state == "GREEN" and self.intersection_detected and self.pending_action in ["LEFT","RIGHT"]):
            self.mode = "TURN"
            self.start_heading = (self.current_heading)
            self.get_logger().info(f"Entering TURN mode: {self.pending_action}")

        # FOLLOW -> STRAIGHT
        elif (self.mode == "FOLLOW"and self.tl_state == "GREEN" and self.intersection_detected and self.pending_action == "STRAIGHT"):
            self.mode = "STRAIGHT"
            self.start_x = self.current_x
            self.start_y = self.current_y
            self.get_logger().info("Entering STRAIGHT mode")

        # FOLLOW -> SPECIAL
        elif (self.mode == "FOLLOW" and self.special_behavior != "NONE"):
            self.mode = "SPECIAL"
            self.special_start_time = (self.current_time)
            self.get_logger().info(f"Entering SPECIAL mode: {self.special_behavior}")

    def handle_turn_mode(self):
        angle_diff = np.arctan2(
            np.sin(self.current_heading - self.start_heading),
            np.cos(self.current_heading - self.start_heading)
        )

        target_angle = np.pi / 2
        remaining_angle = max(0.0, target_angle - abs(angle_diff))

        # velocidad angular progresiva
        angular_speed = 2.0 * remaining_angle
        angular_speed = np.clip(angular_speed, 0.2, 1.0)

        # GIRO SOBRE SU EJE
        self.twist.linear.x = 0.0
        if self.pending_action == "LEFT":
            self.twist.angular.z = angular_speed

        elif self.pending_action == "RIGHT":
            self.twist.angular.z = -angular_speed

        # TERMINAR GIRO
        angle_tolerance = np.deg2rad(3)
        if abs(angle_diff) >= (target_angle - angle_tolerance):
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0

            self.pending_action = "NONE"
            self.mode = "FOLLOW"
            self.get_logger().info("Turn completed")
            self.prev_error = 0.0
            self.line_error = 0.0

    def handle_straight_mode(self):
        self.twist.linear.x = 0.10
        self.twist.angular.z = 0.0
        distance = np.sqrt((self.current_x - self.start_x)**2 + (self.current_y - self.start_y)**2)

        if distance >= self.target_distance:
            self.pending_action = "NONE"
            self.mode = "FOLLOW"
            self.get_logger().info("Straight completed")

    def handle_stop_mode(self):
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0

    def handle_special_mode(self):
        speed_multiplier = 1.0
        elapsed = (self.current_time - self.special_start_time)
        if self.special_behavior == "WORKERS":
            speed_multiplier = 0.5

        elif self.special_behavior == "GIVE_WAY":
            speed_multiplier = 0.6

        elif self.special_behavior == "ROUND":
            speed_multiplier = 0.7

        self.handle_follow_mode(speed_multiplier)

        if elapsed >= self.special_duration:
            self.special_behavior = "NONE"
            self.mode = "FOLLOW"
            self.get_logger().info("Special mode completed")

    def handle_follow_mode(self,speed_multiplier=1.0):

        # CURVE DETECTION
        curve_detected = (abs(self.line_error) > self.curve_threshold)
        if curve_detected != self.curve_state:
            if curve_detected:
                self.get_logger().info("↩️ Curva detectada")
            else:
                self.get_logger().info("➡️ Recta detectada")
            self.curve_state = curve_detected
    
        # ADAPTIVE KP
        if curve_detected:
            kp = self.kp_curve
        else:
            kp = self.kp_straight

        # PD Controller
        derivative = self.line_error - self.prev_error
        derivative = np.clip(derivative, -70, 70)

        angular_vel = (kp * self.line_error + self.kd * derivative)
        self.prev_error = self.line_error

        # Saturation
        angular_vel = np.clip(angular_vel,-self.w_max,self.w_max)
        angular_vel *= 0.85

        # Adaptive Linear Speed
        if curve_detected:
            target_linear_vel = self.curve_speed
        else:
            target_linear_vel = self.linear_speed

        # -------- Acceleration Ramp --------
        if self.current_linear_vel < target_linear_vel:
            self.current_linear_vel += self.acceleration
            self.current_linear_vel = min(self.current_linear_vel, target_linear_vel)

        elif self.current_linear_vel > target_linear_vel:
            self.current_linear_vel -= self.acceleration
            self.current_linear_vel = max(self.current_linear_vel,target_linear_vel)

        #LINE LOST
        linear_vel = self.current_linear_vel
        if abs(self.line_error) > self.line_lost_threshold:
            linear_vel *= 0.4

        # TURN-BASED SPEED REDUCTION
        turn_factor = 1.0 - min(abs(angular_vel) / self.w_max,1.0)
        turn_factor = max(turn_factor,0.35)
        linear_vel *= turn_factor

        # SPECIAL MODIFIER
        linear_vel *= speed_multiplier

        if self.tl_state == "RED":
            linear_vel = 0.0
        elif self.tl_state == "YELLOW":
            linear_vel *= 0.5

        self.twist.linear.x = linear_vel
        self.twist.angular.z = angular_vel

    def _control_loop(self):
        self.twist = Twist()
        self.current_time = (self.get_clock().now().nanoseconds / 1e9)

        # FSM
        self.update_fsm()

        # MODES
        if self.mode == "TURN":
            self.handle_turn_mode()

        elif self.mode == "STRAIGHT":
            self.handle_straight_mode()

        elif self.mode == "SPECIAL":
            self.handle_special_mode()

        elif self.mode == "STOPPED":
            self.handle_stop_mode()

        else:
            self.handle_follow_mode()

        motion_state = ""
        if self.tl_state == "RED":
            motion_state = "🔴 Rojo detectado -> detenido"
        elif self.tl_state == "YELLOW":
            motion_state = "🟡 Amarillo detectado -> reduciendo velocidad"
        elif self.tl_state == "GREEN":
            if self.twist.linear.x > 0.01:
                motion_state = f"🟢 Verde -> avanzando ({self.twist.linear.x:.2f} m/s)"
            else:
                motion_state = "🟢 Verde detectado -> alineando"
        else:
            motion_state = "⚪ Buscando semaforo"

        # Imprimir solo si cambia el estado
        if motion_state != self.last_motion_state:
            self.get_logger().info(motion_state)
            self.last_motion_state = motion_state

        # publish
        self.pub_vel.publish(self.twist)

def main(args=None):
    rclpy.init(args=args)
    node = LineFollowerController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()