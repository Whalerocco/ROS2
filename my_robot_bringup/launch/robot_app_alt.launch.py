from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    robot_names = ["Giskard", "BB8", "Daneel", "Jander", "C3PO"]

    robot_news_station_nodes = []

    for name in robot_names:
        robot_news_station_nodes.append(Node(
            package = "my_cpp_pkg",
            executable = "robot_news_station",
            name = "robot_news_station_" + name.lower(),
            parameters = [
                {"robot_name": name}
            ]
        ))

    phone_node = Node(
        package = "my_cpp_pkg",
        executable = "smartphone",
        name = "Nokia_3310"
    )

    for node in robot_news_station_nodes:
        ld.add_action(node)

    ld.add_action(phone_node)
    return ld