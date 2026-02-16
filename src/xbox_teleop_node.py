#!/usr/bin/env python3
"""
ROS 2 Teleop node for 4-wheeled Red Rover using Xbox 360 controller.

This node subscribes to sensor_msgs/Joy from the joy_node and publishes
geometry_msgs/Twist commands for differential drive control.

Control Scheme:
    Left Stick Y:   Forward/Backward (linear velocity)
    Left Stick X:   Turn left/right (angular velocity)
    LT + RT (hold): Turbo mode (full speed)
    A button:       Arm motors
    B button:       Disarm motors (emergency stop)

Safety Features:
    - Starts disarmed (must press A to enable movement)
    - Normal mode: 50% max speed at full stick deflection
    - Turbo mode (hold both triggers): 100% max speed
    - Immediate zero on release of stick
    - Zero command sent on shutdown
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
import time


class XboxTeleopNode(Node):
    """ROS 2 teleop node for Xbox 360 controller."""

    # Xbox 360 controller mapping
    # Note: These may vary by driver. We'll log actual values to debug.
    AXIS_LEFT_X = 0      # Left Stick X (turn)
    AXIS_LEFT_Y = 1      # Left Stick Y (forward/back)
    AXIS_LEFT_TRIGGER = 2   # LT trigger
    AXIS_RIGHT_TRIGGER = 5  # RT trigger
    
    BUTTON_A = 0         # Arm
    BUTTON_B = 1         # Disarm
    BUTTON_LB = 4        # Dead man's switch

    def __init__(self):
        super().__init__('xbox_teleop_node')
        
        # Declare parameters
        self.declare_parameter('max_linear_vel', 1.0)   # m/s (matches Teensy VMAX)
        self.declare_parameter('max_angular_vel', 1.5)  # rad/s (matches Teensy OMEGAMAX)
        self.declare_parameter('normal_scale', 1.0)  # speed multiplier for normal mode
        self.declare_parameter('turbo_scale', 1.0)   # speed multiplier when LT+RT held
        self.declare_parameter('deadzone', 0.15)
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('debug', True)  # Enable debug logging
        
        # Get parameters
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        self.normal_scale = self.get_parameter('normal_scale').value
        self.turbo_scale = self.get_parameter('turbo_scale').value
        self.deadzone = self.get_parameter('deadzone').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.debug = self.get_parameter('debug').value
        
        # State
        self.armed = False
        self.turbo = False
        self.deadman_pressed = False
        self.prev_deadman_pressed = False  # For edge detection
        
        # Controller state
        self.target_linear = 0.0
        self.target_angular = 0.0
        
        # Debug: count messages received
        self.joy_msg_count = 0
        self.last_joy_time = None
        
        # Publishers and subscribers
        # Use RELIABLE for cmd_vel to ensure proper bridging to ROS1
        qos_reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        qos_best_effort = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos_reliable)
        self.arm_pub = self.create_publisher(Bool, 'arm', qos_reliable)
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, qos_best_effort)
        
        # Timer for publishing at fixed rate
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.publish_cmd_vel)
        
        # Debug timer - periodic status
        self.debug_timer = self.create_timer(2.0, self.debug_status)
        
        self.get_logger().info('=' * 50)
        self.get_logger().info('Xbox Teleop Node started')
        self.get_logger().info('=' * 50)
        self.get_logger().info('Controls:')
        self.get_logger().info('  Left Stick: Move forward/back and turn')
        self.get_logger().info('  LT + RT (hold both): Turbo mode')
        self.get_logger().info('  A: Arm motors')
        self.get_logger().info('  B: Disarm motors')
        self.get_logger().info('')
        self.get_logger().info(f'Max speeds: {self.max_linear_vel} m/s linear, {self.max_angular_vel} rad/s angular')
        self.get_logger().info(f'Normal mode: {self.normal_scale*100:.0f}% speed | Turbo (LT+RT): {self.turbo_scale*100:.0f}% speed')
        self.get_logger().info('Press A to arm, then use left stick to drive.')
        self.get_logger().info(f'Debug mode: {self.debug}')
        self.get_logger().info('Status: DISARMED')
        self.get_logger().info('=' * 50)

    def debug_status(self):
        """Periodically log debug status."""
        if not self.debug:
            return
            
        armed_str = "ARMED" if self.armed else "DISARMED"
        mode_str = "TURBO" if self.turbo else "NORMAL"
        
        self.get_logger().info(
            f'[STATUS] {armed_str} | {mode_str} | '
            f'Joy msgs: {self.joy_msg_count} | '
            f'Target: lin={self.target_linear:.3f} ang={self.target_angular:.3f}'
        )

    def apply_deadzone(self, value: float) -> float:
        """Apply deadzone to axis value."""
        if abs(value) < self.deadzone:
            return 0.0
        # Scale remaining range to 0-1
        sign = 1.0 if value > 0 else -1.0
        return sign * (abs(value) - self.deadzone) / (1.0 - self.deadzone)

    def joy_callback(self, msg: Joy):
        """Process joystick input."""
        self.joy_msg_count += 1
        self.last_joy_time = time.time()
        
        # Log first message and periodically after
        if self.joy_msg_count == 1:
            self.get_logger().info(f'First joy message received!')
            self.get_logger().info(f'  Axes count: {len(msg.axes)}')
            self.get_logger().info(f'  Buttons count: {len(msg.buttons)}')
            self.get_logger().info(f'  Axes: {list(msg.axes)}')
            self.get_logger().info(f'  Buttons: {list(msg.buttons)}')
        
        # Ensure we have enough axes and buttons
        if len(msg.axes) < 6 or len(msg.buttons) < 6:
            self.get_logger().warn(
                f'Joy message has insufficient axes ({len(msg.axes)}) or buttons ({len(msg.buttons)})',
                throttle_duration_sec=5.0
            )
            return
        
        # Log raw values periodically for debugging
        if self.debug and self.joy_msg_count % 100 == 0:
            self.get_logger().info(
                f'[RAW] Axes: LX={msg.axes[0]:.2f} LY={msg.axes[1]:.2f} | '
                f'Buttons: A={msg.buttons[0]} B={msg.buttons[1]} LB={msg.buttons[4]}'
            )
        
        # Check button presses
        a_pressed = msg.buttons[self.BUTTON_A] == 1
        b_pressed = msg.buttons[self.BUTTON_B] == 1
        
        # Arm/Disarm (with logging)
        # Note: We also publish to the /arm topic for the Teensy firmware
        if a_pressed and not self.armed:
            self.armed = True
            arm_msg = Bool()
            arm_msg.data = True
            self.arm_pub.publish(arm_msg)
            self.get_logger().info('>>> Motors ARMED (published to /arm topic) <<<')
        elif b_pressed and self.armed:
            self.armed = False
            arm_msg = Bool()
            arm_msg.data = False
            self.arm_pub.publish(arm_msg)
            self.get_logger().info('>>> Motors DISARMED (published to /arm topic) <<<')
        
        # Turbo mode: hold both LT and RT triggers
        lt_pressed = msg.axes[self.AXIS_LEFT_TRIGGER] < 0.0
        rt_pressed = msg.axes[self.AXIS_RIGHT_TRIGGER] < 0.0
        prev_turbo = self.turbo
        self.turbo = lt_pressed and rt_pressed
        
        if self.turbo and not prev_turbo:
            self.get_logger().info('>>> TURBO MODE ON (LT+RT held) <<<')
        elif not self.turbo and prev_turbo:
            self.get_logger().info('>>> TURBO MODE OFF <<<')
        
        # Get stick values
        # Invert Y axis so pushing stick UP = positive (forward) velocity
        raw_forward = msg.axes[self.AXIS_LEFT_Y]
        raw_turn = msg.axes[self.AXIS_LEFT_X]
        forward = raw_forward
        turn = -raw_turn
        
        # Apply deadzone
        forward_dz = self.apply_deadzone(forward)
        turn_dz = self.apply_deadzone(turn)
        
        # Log significant stick movements
        if self.debug and (abs(forward_dz) > 0.01 or abs(turn_dz) > 0.01):
            self.get_logger().info(
                f'[STICK] Raw: fwd={raw_forward:.2f} turn={raw_turn:.2f} | '
                f'After deadzone: fwd={forward_dz:.2f} turn={turn_dz:.2f}',
                throttle_duration_sec=0.5
            )
        
        # Compute target velocities with speed scaling
        if self.armed:
            scale = self.turbo_scale if self.turbo else self.normal_scale
            self.target_linear = forward_dz * self.max_linear_vel * scale
            self.target_angular = -turn_dz * self.max_angular_vel * scale
            
            # Log non-zero commands
            if self.debug and (abs(self.target_linear) > 0.01 or abs(self.target_angular) > 0.01):
                self.get_logger().info(
                    f'[CMD] Target velocities: linear={self.target_linear:.3f} angular={self.target_angular:.3f}',
                    throttle_duration_sec=0.5
                )
        else:
            # Ramp to zero
            if self.target_linear != 0.0 or self.target_angular != 0.0:
                if self.debug:
                    self.get_logger().info('[CMD] Setting target to zero (not armed)')
            self.target_linear = 0.0
            self.target_angular = 0.0
        
        # Clamp to limits
        self.target_linear = max(-self.max_linear_vel, 
                                  min(self.max_linear_vel, self.target_linear))
        self.target_angular = max(-self.max_angular_vel, 
                                   min(self.max_angular_vel, self.target_angular))

    def publish_cmd_vel(self):
        """Publish velocity command."""
        twist = Twist()
        
        if self.armed:
            twist.linear.x = self.target_linear
            twist.angular.z = self.target_angular
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        
        self.cmd_vel_pub.publish(twist)
        
        # Log when publishing non-zero velocities (throttled)
        if self.debug and (abs(twist.linear.x) > 0.01 or abs(twist.angular.z) > 0.01):
            self.get_logger().info(
                f'[PUB] Publishing cmd_vel: linear={twist.linear.x:.3f} angular={twist.angular.z:.3f}',
                throttle_duration_sec=0.5
            )

    def shutdown(self):
        """Send zero velocity and disarm on shutdown."""
        self.get_logger().info('Shutting down, sending zero velocity and disarming')
        
        # Send zero velocity
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        
        # Disarm the Teensy
        arm_msg = Bool()
        arm_msg.data = False
        self.arm_pub.publish(arm_msg)


def main(args=None):
    rclpy.init(args=args)
    
    node = XboxTeleopNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
