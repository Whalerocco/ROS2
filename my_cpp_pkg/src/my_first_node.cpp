#include "rclcpp/rclcpp.hpp"

// Everything that happens in the node is written in the node class:
class MyNode : public rclcpp::Node /* Could be a robot controller node for example*/
{
public:
    MyNode() : Node("cpp_test"), counter_(0)                            // A node with cpp_test as the name
    {                                                      // Constructor below
        RCLCPP_INFO(this->get_logger(), "Hello Cpp Nöde"); /*Use rclcpp to print something from the node, instead of node it's this because get_logger is now part of the (node) class*/

        // Calls timerCallback every 1 sec
        timer_ = this->create_wall_timer(std::chrono::seconds(1),
                                         std::bind(&MyNode::timerCallback, this)); // The timerCallback method is bound to the MyNode class/node
    }

private:
    void timerCallback() // Method called by the timer
    {
        counter_++;
        RCLCPP_INFO(this->get_logger(), "Hello %d", counter_);
    }

    // Declare the timer as a private member of the class
    rclcpp::TimerBase::SharedPtr timer_;
    int counter_; // Attribute
};

// The main function will just contain of these 5 lines of (uncomment) code
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv); // init ros2 communications

    // auto node = std::make_shared<rclcpp::Node>("cpp_test"); /*read about shared nodes */
    // RCLCPP_INFO(node->get_logger(), "Hello Cpp Node");
    auto node = std::make_shared<MyNode>(); // Create a shared pointed to Node. No arg needed, everything will be inside the node class

    rclcpp::spin(node); /* Pauses the program here but keeps it alive, stops spinning at ctrl+c*/
    rclcpp::shutdown(); /* shuts down and destroys node*/

    return 0;
}