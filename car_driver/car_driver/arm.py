#!/usr/bin/env python3
"""Helpers to (re-)arm autonomous, plus a one-shot CLI.

Import the helpers from your own nodes instead of re-implementing the service
call:

    from car_driver.arm import arm_async        # inside a spinning node
    from car_driver.arm import arm_blocking      # inside a plain script

Or run the one-shot CLI:

    ros2 run car_driver arm
"""

import sys

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


DEFAULT_ENGAGE_SERVICE = "/rc_car_driver/engage_autonomous"


def _log_result(node: Node, future) -> bool:
    result = future.result()
    if result is None:
        node.get_logger().error("No response from engage service.")
        return False
    if result.success:
        node.get_logger().info(f"Armed: {result.message}")
    else:
        node.get_logger().warning(f"Arm refused: {result.message}")
    return result.success


def arm_async(
    node: Node,
    service_name: str = DEFAULT_ENGAGE_SERVICE,
    wait_timeout: float = 5.0,
    done_callback=None,
):
    """Request an arm without blocking; safe to call from a spinning node.

    Returns the future (or None if the service never showed up). If no
    done_callback is given, the result is logged on `node`.
    """
    client = node.create_client(Trigger, service_name)

    if not client.wait_for_service(timeout_sec=wait_timeout):
        node.get_logger().warning(
            f"Engage service '{service_name}' unavailable; "
            "autonomous may stay disarmed. Arm it manually."
        )
        return None

    future = client.call_async(Trigger.Request())
    future.add_done_callback(
        done_callback if done_callback is not None
        else (lambda fut: _log_result(node, fut))
    )
    return future


def arm_blocking(
    node: Node,
    service_name: str = DEFAULT_ENGAGE_SERVICE,
    timeout: float = 5.0,
) -> bool:
    """Request an arm and wait for the result. Do NOT call from a node that is
    already being spun elsewhere; use arm_async there instead.
    """
    client = node.create_client(Trigger, service_name)

    if not client.wait_for_service(timeout_sec=timeout):
        node.get_logger().error(
            f"Service '{service_name}' unavailable. Is driver_node running?"
        )
        return False

    future = client.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    return _log_result(node, future)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Node("arm_autonomous_client")
    node.declare_parameter("engage_service", DEFAULT_ENGAGE_SERVICE)
    service_name = str(node.get_parameter("engage_service").value)

    try:
        success = arm_blocking(node, service_name)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
