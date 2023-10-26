from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    remap_nr_topic = ("numberTopic", "number")
    
    number_publisher_node = Node(
        package = "my_cpp_pkg",
        executable = "number_publisher",
        name = "your_number_publisher",
        remappings=[
            remap_nr_topic
        ],
        parameters=[
            {"mult": 2},
            {"init_number": 50}
        ]
    )

    number_counter_node = Node(
        package = "my_cpp_pkg",
        executable = "number_counter",
        name = "your_number_counter",
        remappings=[
            remap_nr_topic,
            ("countTopic", "count")
        ]
    )

    ld.add_action(number_publisher_node)
    ld.add_action(number_counter_node)
    
    return ld