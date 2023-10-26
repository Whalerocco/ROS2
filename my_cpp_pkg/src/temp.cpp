#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/srv/set_led.hpp"

// Toggles battery status every 6/10s between full and empty
// At the toggle, send a request to the led_panel_server with the set_led.srv

class BatteryNode : public rclcpp::Node
{
public:
    BatteryNode() : Node("add_two_ints_client"), batteryLevel(false)
    {
        // callAddTwoIntsService(1,5);
        // The callback needs to be called in another threadd, otherwise this node will get stuck
        // thread1  = std::thread(std::bind(&AddTwoIntsClientNode::callAddTwoIntsService, this, 1, 4));

        // To send multiple calls to the server, a thread pool can be implemented.
        threads.push_back(std::thread(std::bind(&BatteryNode::callBatteryStatus, this)));

        // timer to update the battery status, cant do anything at the same time if using this method.
        timer = this->create_wall_timer(
            std::chrono::milliseconds(3400),
            std::bind(&BatteryNode::callBatteryStatus, this));
    }

    void updateBatteryLevel()
    {
        batteryLevel = (batteryLevel + 1) % 4;
    }

    void callBatteryStatus()
    {
        updateBatteryLevel();
        auto client = this->create_client<my_robot_interfaces::srv::SetLed>("set_led");
        while (rclcpp::ok())
        {

            if (client->wait_for_service(std::chrono::seconds(1))){
                // create a shared pointer
                auto request = std::make_shared<my_robot_interfaces::srv::SetLed::Request>();
                if (batteryLevel == 0)
                {
                    request->lednumber = 0;
                    request->state = false;
                }
                else if (batteryLevel == 1)
                {
                    request->lednumber = 1;
                    request->state = true;
                }
                else if (batteryLevel == 2)
                {
                    request->lednumber = 2;
                    request->state = true;
                }
                else if (batteryLevel == 3)
                {
                    request->lednumber = 3;
                    request->state = true;
                }

                auto future = client->async_send_request(request); // Sends request to the server

                // Blocks until a response from the future, can throw eerror  wheen exception
                try
                {
                    auto response = future.get();
                    if (response->success)
                    {
                        RCLCPP_INFO(this->get_logger(), "Successfully changed the led");
                    }
                    else
                    {
                        RCLCPP_INFO(this->get_logger(), "Couldn't change the led");
                    }
                }
                catch (const std::exception &e)
                {
                    RCLCPP_ERROR(this->get_logger(), "Service call failed");
                }
            }
            else{
                RCLCPP_WARN(this->get_logger(), "Waiting for the server to be up...");
            }
        }
    }


private:
    // Thread object needs to be a member of the class
    std::thread thread1;
    std::vector<std::thread> threads;
    rclcpp::TimerBase::SharedPtr timer;

    int batteryLevel; // 0 is empty, 3 is full

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BatteryNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}