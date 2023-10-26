#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"
#include <stdlib.h>
#include <time.h>

class NumberPublisherNode : public rclcpp::Node  
{
public:
    NumberPublisherNode() : Node("number_publisher")
    {
        this->declare_parameter("mult", 1);
        this->declare_parameter("init_number", 1);

        multiplier = this->get_parameter("mult").as_int();
        number = this->get_parameter("init_number").as_int();
        
        RCLCPP_INFO(this->get_logger(), "init_number parameter: %d\n", number);
        //Initialize publisher
        nbr_publisher = this->create_publisher<example_interfaces::msg::Int64>("numberTopic", 10);
        //Init timer
        number_timer = this->create_wall_timer(std::chrono::milliseconds(1000),std::bind(&NumberPublisherNode::publishNumber, this));
        randNumber_timer = this->create_wall_timer(std::chrono::milliseconds(1000),std::bind(&NumberPublisherNode::generateRandNumber, this));
        RCLCPP_INFO(this->get_logger(), "Number publisher has started");
        srand(time(NULL));
    }

private:
    
    void publishNumber()
    {
        //Create the message object
        auto msg = example_interfaces::msg::Int64(); // Why is () needed here?
        //Fill the data field
        msg.data = number;
        //publish message
        nbr_publisher->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Number = %d sent", number);
    }

    void generateRandNumber()
    {
        number = (rand() % 10 +1)*multiplier;
    }
    
    //Private Attributes
    int number;
    int multiplier;
    //Declare the publisher
    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr nbr_publisher;
    //Declare the timer
    rclcpp::TimerBase::SharedPtr number_timer;
    rclcpp::TimerBase::SharedPtr randNumber_timer;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberPublisherNode>();  
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}