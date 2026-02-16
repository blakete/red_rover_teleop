#!/usr/bin/env python3
"""
Teleop control for 4-wheeled Red Rover using Xbox 360 controller.

This script reads Xbox 360 controller inputs directly (no ROS required) and
converts them to motor commands for a 4-wheeled skid-steer differential drive rover.

Control Scheme:
    Left Stick Y:   Forward/Backward (linear velocity)
    Left Stick X:   Turn left/right (angular velocity)
    Right Trigger:  Turbo mode (faster speeds when held)
    LB (Left Bumper): Dead man's switch (must hold to move)
    A button:       Arm motors
    B button:       Disarm motors (emergency stop)
    X button:       Toggle verbose output
    Start button:   Exit program

Motor Layout (viewed from above):
    Front Left (FL)   ----   Front Right (FR)
          |                       |
          |      [ROVER]          |
          |                       |
    Rear Left (RL)    ----   Rear Right (RR)

For skid-steer drive:
    Left wheels:  v_left  = v_linear - (v_angular * wheel_base / 2)
    Right wheels: v_right = v_linear + (v_angular * wheel_base / 2)

Usage:
    python teleop_control_xbox360.py                    # Use default device
    python teleop_control_xbox360.py /dev/input/js1    # Specify device
    python teleop_control_xbox360.py --serial /dev/ttyUSB0  # Output to serial
"""

import struct
import argparse
import time
import sys
from typing import Optional, Tuple, Callable
from dataclasses import dataclass


# =============================================================================
# Xbox 360 Controller Mapping
# =============================================================================

AXIS_MAP = {
    0: "left_x",      # Left Stick X
    1: "left_y",      # Left Stick Y
    2: "left_trigger",
    3: "right_x",     # Right Stick X
    4: "right_y",     # Right Stick Y
    5: "right_trigger",
    6: "dpad_x",
    7: "dpad_y",
}

BUTTON_MAP = {
    0: "a",
    1: "b",
    2: "x",
    3: "y",
    4: "lb",
    5: "rb",
    6: "back",
    7: "start",
    8: "xbox",
    9: "left_stick_click",
    10: "right_stick_click",
}


# =============================================================================
# Configuration
# =============================================================================

@dataclass
class RoverConfig:
    """Configuration for the Red Rover."""
    # Velocity limits (m/s or rad/s)
    max_linear_vel: float = 1.0      # m/s
    max_angular_vel: float = 1.5     # rad/s
    
    # Speed modes
    low_linear_speed: float = 0.4    # m/s in normal mode
    high_linear_speed: float = 1.0   # m/s in turbo mode
    low_angular_speed: float = 0.6   # rad/s in normal mode
    high_angular_speed: float = 1.2  # rad/s in turbo mode
    
    # Deadzone (ignore stick inputs below this threshold)
    deadzone: float = 0.1
    
    # Smoothing (alpha filter time constant)
    alpha_tau_linear: float = 0.15   # seconds
    alpha_tau_angular: float = 0.15  # seconds
    
    # Control loop rate
    loop_rate_hz: float = 50.0       # Hz
    
    # Physical dimensions (for differential drive calculation)
    wheel_base: float = 0.4          # meters (track width)


@dataclass 
class ControllerState:
    """Current state of the Xbox 360 controller."""
    # Axes (normalized -1.0 to 1.0)
    left_x: float = 0.0
    left_y: float = 0.0
    right_x: float = 0.0
    right_y: float = 0.0
    left_trigger: float = 0.0
    right_trigger: float = 0.0
    dpad_x: float = 0.0
    dpad_y: float = 0.0
    
    # Buttons (True = pressed)
    a: bool = False
    b: bool = False
    x: bool = False
    y: bool = False
    lb: bool = False
    rb: bool = False
    back: bool = False
    start: bool = False
    xbox: bool = False
    left_stick_click: bool = False
    right_stick_click: bool = False


@dataclass
class MotorCommand:
    """Motor velocities for 4-wheeled rover."""
    front_left: float = 0.0
    front_right: float = 0.0
    rear_left: float = 0.0
    rear_right: float = 0.0
    
    def __str__(self) -> str:
        return (f"FL:{self.front_left:+6.3f} FR:{self.front_right:+6.3f} "
                f"RL:{self.rear_left:+6.3f} RR:{self.rear_right:+6.3f}")
    
    def to_bytes(self) -> bytes:
        """Pack motor commands as 4 signed 16-bit integers."""
        # Scale from velocity to motor units (-32767 to 32767)
        scale = 32767
        fl = int(max(-scale, min(scale, self.front_left * scale)))
        fr = int(max(-scale, min(scale, self.front_right * scale)))
        rl = int(max(-scale, min(scale, self.rear_left * scale)))
        rr = int(max(-scale, min(scale, self.rear_right * scale)))
        return struct.pack('<4h', fl, fr, rl, rr)


# =============================================================================
# Teleop Controller
# =============================================================================

class TeleopController:
    """Main teleop controller for the Red Rover."""
    
    def __init__(self, 
                 device_path: str = "/dev/input/js0",
                 config: Optional[RoverConfig] = None,
                 output_callback: Optional[Callable[[MotorCommand], None]] = None):
        """
        Initialize the teleop controller.
        
        Args:
            device_path: Path to joystick device
            config: Rover configuration
            output_callback: Function to call with motor commands
        """
        self.device_path = device_path
        self.config = config or RoverConfig()
        self.output_callback = output_callback or self._default_output
        
        self.controller = ControllerState()
        self.armed = False
        self.verbose = True
        self.running = True
        
        # For smoothing
        self.last_linear = 0.0
        self.last_angular = 0.0
        self.last_time = time.time()
        
        # Event struct format
        self.event_format = "IhBB"
        self.event_size = struct.calcsize(self.event_format)
        
    def _default_output(self, cmd: MotorCommand) -> None:
        """Default output: print motor commands."""
        armed_str = "ARMED  " if self.armed else "DISARMED"
        print(f"\r[{armed_str}] {cmd}", end="", flush=True)
    
    def _apply_deadzone(self, value: float) -> float:
        """Apply deadzone to axis value."""
        if abs(value) < self.config.deadzone:
            return 0.0
        # Scale remaining range to 0-1
        sign = 1.0 if value > 0 else -1.0
        return sign * (abs(value) - self.config.deadzone) / (1.0 - self.config.deadzone)
    
    def _alpha_filter(self, new_val: float, old_val: float, dt: float, tau: float) -> float:
        """Apply exponential smoothing (alpha filter)."""
        if tau <= 0:
            return new_val
        alpha = dt / (dt + tau)
        return alpha * new_val + (1 - alpha) * old_val
    
    def _compute_velocities(self) -> Tuple[float, float]:
        """Compute linear and angular velocities from controller state."""
        # Get raw stick values
        forward = -self.controller.left_y  # Invert Y (up = positive)
        turn = self.controller.left_x
        
        # Apply deadzone
        forward = self._apply_deadzone(forward)
        turn = self._apply_deadzone(turn)
        
        # Check for turbo mode (right trigger or RB button)
        turbo = self.controller.right_trigger > 0.5 or self.controller.rb
        
        if turbo:
            linear_scale = self.config.high_linear_speed
            angular_scale = self.config.high_angular_speed
        else:
            linear_scale = self.config.low_linear_speed
            angular_scale = self.config.low_angular_speed
        
        # Scale to velocity
        linear_vel = forward * linear_scale
        angular_vel = turn * angular_scale
        
        # Clamp to limits
        linear_vel = max(-self.config.max_linear_vel, 
                        min(self.config.max_linear_vel, linear_vel))
        angular_vel = max(-self.config.max_angular_vel, 
                         min(self.config.max_angular_vel, angular_vel))
        
        return linear_vel, angular_vel
    
    def _differential_to_motor(self, linear: float, angular: float) -> MotorCommand:
        """Convert differential drive commands to individual motor velocities."""
        # For skid-steer drive:
        # v_left = v_linear - v_angular * (wheel_base / 2)
        # v_right = v_linear + v_angular * (wheel_base / 2)
        
        half_base = self.config.wheel_base / 2.0
        
        v_left = linear - angular * half_base
        v_right = linear + angular * half_base
        
        # Normalize if exceeding limits
        max_vel = max(abs(v_left), abs(v_right))
        if max_vel > self.config.max_linear_vel:
            v_left = v_left / max_vel * self.config.max_linear_vel
            v_right = v_right / max_vel * self.config.max_linear_vel
        
        # All left wheels get same velocity, all right wheels get same velocity
        return MotorCommand(
            front_left=v_left,
            front_right=v_right,
            rear_left=v_left,
            rear_right=v_right
        )
    
    def _process_event(self, event: bytes) -> bool:
        """
        Process a joystick event.
        
        Returns True if the event was handled, False on exit signal.
        """
        timestamp, value, event_type, number = struct.unpack(self.event_format, event)
        
        # Ignore init events (bit 0x80)
        if event_type & 0x80:
            return True
        
        if event_type == 1:  # Button
            pressed = bool(value)
            button_name = BUTTON_MAP.get(number, f"button_{number}")
            
            if hasattr(self.controller, button_name):
                setattr(self.controller, button_name, pressed)
            
            # Handle special buttons on press
            if pressed:
                if button_name == "a":  # Arm
                    self.armed = True
                    if self.verbose:
                        print("\n[INFO] Motors ARMED")
                elif button_name == "b":  # Disarm
                    self.armed = False
                    if self.verbose:
                        print("\n[INFO] Motors DISARMED")
                elif button_name == "x":  # Toggle verbose
                    self.verbose = not self.verbose
                    print(f"\n[INFO] Verbose output: {'ON' if self.verbose else 'OFF'}")
                elif button_name == "start":  # Exit
                    print("\n[INFO] Exit requested")
                    return False
                    
        elif event_type == 2:  # Axis
            # Normalize to -1.0 to 1.0
            normalized = value / 32767.0
            axis_name = AXIS_MAP.get(number, f"axis_{number}")
            
            if hasattr(self.controller, axis_name):
                setattr(self.controller, axis_name, normalized)
        
        return True
    
    def update(self) -> MotorCommand:
        """
        Compute current motor command based on controller state.
        
        Returns:
            MotorCommand with individual motor velocities
        """
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        # Check dead man's switch (LB button)
        if not self.controller.lb or not self.armed:
            # Smoothly ramp down to zero
            target_linear = 0.0
            target_angular = 0.0
        else:
            target_linear, target_angular = self._compute_velocities()
        
        # Apply smoothing
        self.last_linear = self._alpha_filter(
            target_linear, self.last_linear, dt, self.config.alpha_tau_linear)
        self.last_angular = self._alpha_filter(
            target_angular, self.last_angular, dt, self.config.alpha_tau_angular)
        
        # Convert to motor commands
        return self._differential_to_motor(self.last_linear, self.last_angular)
    
    def run(self) -> None:
        """Main control loop."""
        print(f"Opening controller at {self.device_path}...")
        print("\nControls:")
        print("  Left Stick:   Move forward/back and turn")
        print("  LB (hold):    Dead man's switch")
        print("  Right Trigger/RB: Turbo mode")
        print("  A:            Arm motors")
        print("  B:            Disarm motors")
        print("  X:            Toggle verbose")
        print("  Start:        Exit")
        print("\nPress A to arm, then hold LB and use left stick to drive.\n")
        
        try:
            with open(self.device_path, "rb") as device:
                # Set non-blocking mode
                import fcntl
                import os
                flags = fcntl.fcntl(device, fcntl.F_GETFL)
                fcntl.fcntl(device, fcntl.F_SETFL, flags | os.O_NONBLOCK)
                
                loop_period = 1.0 / self.config.loop_rate_hz
                
                while self.running:
                    loop_start = time.time()
                    
                    # Read all available events
                    while True:
                        try:
                            event = device.read(self.event_size)
                            if not event:
                                break
                            if not self._process_event(event):
                                self.running = False
                                break
                        except BlockingIOError:
                            break
                    
                    if not self.running:
                        break
                    
                    # Compute and output motor commands
                    cmd = self.update()
                    
                    if self.verbose:
                        self.output_callback(cmd)
                    
                    # Sleep for remainder of loop period
                    elapsed = time.time() - loop_start
                    if elapsed < loop_period:
                        time.sleep(loop_period - elapsed)
                        
        except FileNotFoundError:
            print(f"\nError: Controller not found at {self.device_path}")
            print("Make sure your Xbox 360 controller is connected.")
            print("Try: ls /dev/input/js*")
            sys.exit(1)
        except PermissionError:
            print(f"\nError: Permission denied for {self.device_path}")
            print(f"Try: sudo chmod a+r {self.device_path}")
            sys.exit(1)
        finally:
            # Send stop command
            stop_cmd = MotorCommand()
            self.output_callback(stop_cmd)
            print("\n\nController stopped. Motors at zero.")


# =============================================================================
# Serial Output Handler
# =============================================================================

class SerialOutput:
    """Handler for outputting motor commands over serial."""
    
    def __init__(self, port: str, baudrate: int = 115200):
        """
        Initialize serial output.
        
        Args:
            port: Serial port path (e.g., /dev/ttyUSB0)
            baudrate: Serial baudrate
        """
        import serial
        self.serial = serial.Serial(port, baudrate, timeout=0.1)
        print(f"Opened serial port {port} at {baudrate} baud")
    
    def send(self, cmd: MotorCommand) -> None:
        """Send motor command over serial."""
        # Protocol: Start byte + 4 motor values (16-bit signed) + checksum
        START_BYTE = 0xAA
        data = cmd.to_bytes()
        checksum = sum(data) & 0xFF
        packet = bytes([START_BYTE]) + data + bytes([checksum])
        self.serial.write(packet)
        
        # Also print
        armed_str = "SERIAL"
        print(f"\r[{armed_str}] {cmd}", end="", flush=True)
    
    def close(self) -> None:
        """Close serial port."""
        self.serial.close()


# =============================================================================
# Main
# =============================================================================

def main():
    parser = argparse.ArgumentParser(
        description="Xbox 360 teleop control for 4-wheeled Red Rover",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    parser.add_argument(
        "device", 
        nargs="?", 
        default="/dev/input/js0",
        help="Joystick device path (default: /dev/input/js0)"
    )
    parser.add_argument(
        "--serial", "-s",
        metavar="PORT",
        help="Send commands over serial port (e.g., /dev/ttyUSB0)"
    )
    parser.add_argument(
        "--baudrate", "-b",
        type=int,
        default=115200,
        help="Serial baudrate (default: 115200)"
    )
    parser.add_argument(
        "--max-linear", "-l",
        type=float,
        default=1.0,
        help="Maximum linear velocity m/s (default: 1.0)"
    )
    parser.add_argument(
        "--max-angular", "-a",
        type=float,
        default=1.5,
        help="Maximum angular velocity rad/s (default: 1.5)"
    )
    parser.add_argument(
        "--wheel-base", "-w",
        type=float,
        default=0.4,
        help="Wheel base/track width in meters (default: 0.4)"
    )
    
    args = parser.parse_args()
    
    # Create configuration
    config = RoverConfig(
        max_linear_vel=args.max_linear,
        max_angular_vel=args.max_angular,
        wheel_base=args.wheel_base
    )
    
    # Create output handler
    serial_handler = None
    output_callback = None
    
    if args.serial:
        try:
            serial_handler = SerialOutput(args.serial, args.baudrate)
            output_callback = serial_handler.send
        except Exception as e:
            print(f"Error opening serial port: {e}")
            sys.exit(1)
    
    # Create and run controller
    teleop = TeleopController(
        device_path=args.device,
        config=config,
        output_callback=output_callback
    )
    
    try:
        teleop.run()
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    finally:
        if serial_handler:
            serial_handler.close()


if __name__ == "__main__":
    main()
