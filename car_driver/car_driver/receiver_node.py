import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
import serial
import time



class ReceiverNode(Node):
    def __init__(self):
        super().__init__('receiver_node')

        # RC commands go on a dedicated topic. driver_node treats this as a
        # high-priority override over the autonomous topic.
        self.declare_parameter('rc_topic', '/rc/drive_cmd')
        # Must match driver_node's max_steering_angle_rad so the human gets the
        # full steering range instead of being clamped to ~1 rad.
        self.declare_parameter('max_steering_angle_rad', 0.45)

        rc_topic = str(self.get_parameter('rc_topic').value)
        self.max_steering_angle_rad = float(
            self.get_parameter('max_steering_angle_rad').value
        )

        self.publisher_ = self.create_publisher(
            AckermannDriveStamped, rc_topic, 10
        )
        port = '/dev/ttyACM0'
        self.baudrate = 115200

        self.serial_port = serial.Serial(port, self.baudrate, timeout=0.01)

        time.sleep(2)
        self.serial_port.reset_input_buffer()

        self.get_logger().info(f'Connected to {port}')

        self.timer = self.create_timer(0.01, self.read_and_publish)

    def read_and_publish(self):
        if self.serial_port.in_waiting == 0:
            return

        try:
            line = self.serial_port.readline().decode('utf-8').strip()

            if not line:
                return

            parts = line.split(',')

            # Expected format: D,throttle,steering
            if len(parts) != 3 or parts[0] != 'D':
                self.get_logger().warning(f'Invalid serial data: {line}')
                return

            throttle = float(parts[1])
            steering = float(parts[2])

            throttle = max(-1.0, min(1.0, throttle))
            steering = max(-1.0, min(1.0, steering))

            msg = AckermannDriveStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'

            # Speed stays normalized (driver maps it to ESC throttle).
            # Steering is scaled to radians so it uses the driver's full range.
            msg.drive.speed = throttle
            msg.drive.steering_angle = steering * self.max_steering_angle_rad

            self.publisher_.publish(msg)

            self.get_logger().debug(
                f'Throttle: {throttle:.3f}, steering: {steering:.3f}'
            )

        except UnicodeDecodeError:
            self.get_logger().warning('Received invalid serial characters')

        except ValueError:
            self.get_logger().warning(f'Could not parse serial data: {line}')

        except serial.SerialException as error:
            self.get_logger().error(f'Serial error: {error}')

    def destroy_node(self):
        if self.serial_port.is_open:
            self.serial_port.close()

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = ReceiverNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
