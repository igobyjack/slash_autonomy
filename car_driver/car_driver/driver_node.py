#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from std_srvs.srv import Trigger
from adafruit_servokit import ServoKit


class RCCarDriver(Node):
    """Convert ROS Ackermann commands into steering-servo and ESC PWM."""

    def __init__(self) -> None:
        super().__init__("rc_car_driver")

        # ROS parameters
        self.declare_parameter("steering_channel", 0)
        self.declare_parameter("esc_channel", 2)

        self.declare_parameter("steering_center_deg", 90.0)
        self.declare_parameter("steering_left_deg", 10.0)
        self.declare_parameter("steering_right_deg", 180.0)
        self.declare_parameter("max_steering_angle_rad", 0.45)

        # These are normalized ServoKit continuous-servo values.
        self.declare_parameter("minimum_forward_throttle", 0.10)
        self.declare_parameter("minimum_reverse_throttle", 0.10)
        self.declare_parameter("maximum_forward_throttle", 0.40)
        self.declare_parameter("maximum_reverse_throttle", 0.25)

        # no encoder: speed is treated as a normalized request.
        self.declare_parameter("maximum_commanded_speed", 1.0)

        self.declare_parameter("command_timeout_sec", 0.5)
        self.declare_parameter("pwm_frequency", 100)

        # --- RC override (priority mux) configuration ---
        # RC is high priority, autonomous is low priority.
        self.declare_parameter("rc_topic", "/rc/drive_cmd")
        self.declare_parameter("autonomous_topic", "/autonomous/drive_cmd")
        # How often the control loop selects a source and drives the hardware.
        self.declare_parameter("control_rate_hz", 50.0)
        # If no RC message arrives within this window, the RC link is
        # considered lost (transmitter off / out of range).
        self.declare_parameter("rc_timeout_sec", 0.3)
        # A stick must move past these deadzones to count as human intent.
        self.declare_parameter("rc_speed_deadzone", 0.05)
        self.declare_parameter("rc_steering_deadzone_rad", 0.02)
        # Whether autonomous is armed at startup. Safer to boot disarmed and
        # require an explicit engage before the car can drive itself.
        self.declare_parameter("autonomous_enabled_on_start", False)

        steering_channel = int(
            self.get_parameter("steering_channel").value
        )
        esc_channel = int(
            self.get_parameter("esc_channel").value
        )
        frequency = int(self.get_parameter("pwm_frequency").value)

        # One ServoKit because frequency is global for the PCA9685.
        self.kit = ServoKit(channels=16, frequency=frequency)

        self.steering = self.kit.servo[steering_channel]
        self.steering.set_pulse_width_range(1000, 2000)
        self.steering.actuation_range = 180

        self.esc = self.kit.continuous_servo[esc_channel]
        self.esc.set_pulse_width_range(1000, 2000)

        # Latest command from each source, with arrival times.
        self.rc_msg = None
        self.rc_time = None
        self.auto_msg = None
        self.auto_time = None

        # Sticky latch: once the human takes over, autonomous stays locked out
        # until it is explicitly re-armed via the engage service. This is the
        # single source of truth for whether autonomous is allowed to drive.
        self.autonomous_enabled = bool(
            self.get_parameter("autonomous_enabled_on_start").value
        )
        self.active_source = "IDLE"

        rc_topic = str(self.get_parameter("rc_topic").value)
        autonomous_topic = str(
            self.get_parameter("autonomous_topic").value
        )

        self.rc_subscription = self.create_subscription(
            AckermannDriveStamped,
            rc_topic,
            self.rc_callback,
            10,
        )
        self.auto_subscription = self.create_subscription(
            AckermannDriveStamped,
            autonomous_topic,
            self.auto_callback,
            10,
        )

        control_rate = float(self.get_parameter("control_rate_hz").value)
        self.control_timer = self.create_timer(
            1.0 / control_rate,
            self.control_loop,
        )

        # Explicit re-arm. Manual override latches autonomous OFF; this is the
        # only way to turn it back ON. Call with:
        #   ros2 service call /rc_car_driver/engage_autonomous std_srvs/srv/Trigger
        self.engage_service = self.create_service(
            Trigger,
            "~/engage_autonomous",
            self.engage_autonomous_callback,
        )

        self.stop_vehicle()
        self.center_steering()

        self.get_logger().info(
            "RC car driver started. ESC neutral, steering centered. "
            f"RC override on '{rc_topic}', autonomous on '{autonomous_topic}'. "
            f"Autonomous {'ENABLED' if self.autonomous_enabled else 'DISARMED'}."
        )

    # Callbacks only store the latest command; the control loop decides which
    # one actually reaches the hardware.
    def rc_callback(self, msg: AckermannDriveStamped) -> None:
        self.rc_msg = msg
        self.rc_time = self.get_clock().now()

    def auto_callback(self, msg: AckermannDriveStamped) -> None:
        self.auto_msg = msg
        self.auto_time = self.get_clock().now()

    def _is_fresh(self, stamp, timeout_sec: float) -> bool:
        if stamp is None:
            return False
        elapsed = (self.get_clock().now() - stamp).nanoseconds / 1e9
        return elapsed < timeout_sec

    def _human_is_commanding(self, msg: AckermannDriveStamped) -> bool:
        speed_deadzone = float(
            self.get_parameter("rc_speed_deadzone").value
        )
        steering_deadzone = float(
            self.get_parameter("rc_steering_deadzone_rad").value
        )
        return (
            abs(float(msg.drive.speed)) > speed_deadzone
            or abs(float(msg.drive.steering_angle)) > steering_deadzone
        )

    def control_loop(self) -> None:
        rc_timeout = float(self.get_parameter("rc_timeout_sec").value)
        auto_timeout = float(
            self.get_parameter("command_timeout_sec").value
        )

        rc_link_alive = self._is_fresh(self.rc_time, rc_timeout)

        # Any human stick movement latches autonomous OFF and keeps it off
        # until it is explicitly re-armed via the engage service.
        if (
            rc_link_alive
            and self.autonomous_enabled
            and self._human_is_commanding(self.rc_msg)
        ):
            self.autonomous_enabled = False
            self.get_logger().warning(
                "Manual override engaged. Autonomous latched OFF; call "
                "the engage_autonomous service to re-arm."
            )

        if self.autonomous_enabled:
            if self._is_fresh(self.auto_time, auto_timeout):
                self.apply_command(self.auto_msg)
                self.set_source("AUTONOMOUS")
            else:
                # Armed but no fresh autonomous command: fail safe.
                self.stop_vehicle()
                self.center_steering()
                self.set_source("AUTONOMOUS_IDLE")
        else:
            # Manual mode (latched). Forward RC while the link is alive.
            if rc_link_alive:
                self.apply_command(self.rc_msg)
                self.set_source("MANUAL")
            else:
                # Transmitter off / out of range: fail safe.
                self.stop_vehicle()
                self.center_steering()
                self.set_source("MANUAL_LINK_LOST")

    def engage_autonomous_callback(self, request, response):
        # Refuse to arm while the human is actively holding the sticks, so a
        # re-arm can't fight a command already in progress.
        rc_timeout = float(self.get_parameter("rc_timeout_sec").value)
        if (
            self._is_fresh(self.rc_time, rc_timeout)
            and self._human_is_commanding(self.rc_msg)
        ):
            response.success = False
            response.message = (
                "Refused: RC sticks are active. Center them, then re-arm."
            )
            self.get_logger().warning(response.message)
            return response

        self.autonomous_enabled = True
        response.success = True
        response.message = "Autonomous armed."
        self.get_logger().info("Autonomous re-armed via engage service.")
        return response

    def set_source(self, source: str) -> None:
        if source != self.active_source:
            self.active_source = source
            self.get_logger().info(f"Control source -> {source}")

    def apply_command(self, msg: AckermannDriveStamped) -> None:
        steering_deg = self.map_steering(float(msg.drive.steering_angle))
        throttle = self.map_throttle(float(msg.drive.speed))
        self.steering.angle = steering_deg
        self.esc.throttle = throttle

    def map_steering(self, steering_angle_rad: float) -> float:
        max_angle = float(
            self.get_parameter("max_steering_angle_rad").value
        )
        center = float(
            self.get_parameter("steering_center_deg").value
        )
        left = float(
            self.get_parameter("steering_left_deg").value
        )
        right = float(
            self.get_parameter("steering_right_deg").value
        )

        steering_angle_rad = max(
            -max_angle,
            min(max_angle, steering_angle_rad),
        )

        normalized = steering_angle_rad / max_angle

        if normalized >= 0.0:
            output = center + normalized * (right - center)
        else:
            output = center + (-normalized) * (left - center)

        return max(0.0, min(180.0, output))

    def map_throttle(self, requested_speed: float) -> float:
        max_speed = float(
            self.get_parameter("maximum_commanded_speed").value
        )

        min_forward = float(
            self.get_parameter("minimum_forward_throttle").value
        )
        max_forward = float(
            self.get_parameter("maximum_forward_throttle").value
        )
        min_reverse = float(
            self.get_parameter("minimum_reverse_throttle").value
        )
        max_reverse = float(
            self.get_parameter("maximum_reverse_throttle").value
        )

        requested_speed = max(
            -max_speed,
            min(max_speed, requested_speed),
        )

        # Explicit neutral zone.
        if math.isclose(requested_speed, 0.0, abs_tol=0.001):
            return 0.0

        normalized = abs(requested_speed) / max_speed

        if requested_speed > 0.0:
            return min_forward + normalized * (
                max_forward - min_forward
            )

        reverse_magnitude = min_reverse + normalized * (
            max_reverse - min_reverse
        )
        return -reverse_magnitude

    def center_steering(self) -> None:
        center = float(
            self.get_parameter("steering_center_deg").value
        )
        self.steering.angle = center

    def stop_vehicle(self) -> None:
        self.esc.throttle = 0.0

    def destroy_node(self) -> bool:
        self.stop_vehicle()
        self.center_steering()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RCCarDriver()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_vehicle()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()