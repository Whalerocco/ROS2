from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    
    turtlesim_node = Node(
        package = "turtlesim",
        executable = "turtlesim_node",
        name = "Turtlesim_node"
    )

    turtle_spawner_node = Node(
        package = "turtle_catch_pkg",
        executable = "turtle_spawner",
        parameters = [
            {"spawn_frequency": 1}
        ]
    )

    turtle_ctrl_node = Node(
        package = "turtle_catch_pkg",
        executable = "turtle_ctrl",
        parameters = [
            {"catch_closest_turtle": True}
        ]
    )

    ld.add_action(turtlesim_node)
    ld.add_action(turtle_spawner_node)
    ld.add_action(turtle_ctrl_node)
    
    return ld