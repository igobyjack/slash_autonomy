from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    receiver_node = Node(
        package="car_driver",
        executable="receiver_node",
        name="receiver_node",
        output="screen",
    )

    driver_node = Node(
        package="car_driver",
        executable="driver_node",
        name="rc_car_driver",
        output="screen",
    )

    return LaunchDescription([receiver_node, driver_node])
