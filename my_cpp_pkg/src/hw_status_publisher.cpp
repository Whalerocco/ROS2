#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/msg/hardware_status.hpp"

class HardwareStatusPublisher : public rclcpp::Node  
{
public:
    HardwareStatusPublisher() : Node("hardware_status_publisher")  
    {
        pub = this->create_publisher<my_robot_interfaces::msg::HardwareStatus>("hardware_status", 10);
        timer = this->create_wall_timer(
            std::chrono::milliseconds(1000), 
            std::bind(&HardwareStatusPublisher::publishHardwareStatus, this));
        RCLCPP_INFO(this->get_logger(), "Hardware Status node started.");
    }

private:
    void publishHardwareStatus()
    {
        auto msg = my_robot_interfaces::msg::HardwareStatus();
        msg.temperature = 58;
        msg.are_motors_ready = false;
        msg.debug_message = "Can't run, too höt";
        pub->publish(msg);
    }

    rclcpp::Publisher<my_robot_interfaces::msg::HardwareStatus>::SharedPtr pub;
    rclcpp::TimerBase::SharedPtr timer;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HardwareStatusPublisher>();  
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}