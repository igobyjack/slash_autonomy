#!/usr/bin/env python3
"""Sample autonomy controller.

Publishes AckermannDriveStamped commands to the autonomous topic that
driver_node consumes. This is a stand-in for a real planner/controller: it
just drives gently forward while sweeping the steering with a sine wave so you
can confirm the autonomy path (and the RC override) works end to end.

Remember driver_node boots DISARMED, so this node calls the engage_autonomous
service on startup. The moment you touch the RC sticks, driver_node latches
autonomous OFF and this node's commands are ignored until you re-arm.
"""

import math

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped

from car_driver.arm import arm_async


class SampleController(Node):
    def __init__(self) -> None:
        super().__init__("sample_controller")

        autonomous_topic = "/autonomous/drive_cmd"
        self.cruise_speed = 0.15
        self.steering_amplitude = 0.3
        self.steering_period = 4.0
        publish_rate = 20.0

        self.publisher = self.create_publisher(
            AckermannDriveStamped,
            autonomous_topic,
            10,
        )

        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(
            1.0 / publish_rate,
            self.publish_command,
        )


        arm_async(self)


    def publish_command(self) -> None:
        elapsed = (
            self.get_clock().now() - self.start_time
        ).nanoseconds / 1e9

        steering = self.steering_amplitude * math.sin(
            2.0 * math.pi * elapsed / self.steering_period
        )

        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = self.cruise_speed
        msg.drive.steering_angle = steering

        self.publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SampleController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
