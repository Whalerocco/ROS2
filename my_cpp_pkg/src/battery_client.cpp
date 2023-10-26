#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/srv/set_led.hpp"
#include <mutex>
#include <condition_variable>
#include <thread>

//using namespace std;

class BatteryNode : public rclcpp::Node
{
public:
    BatteryNode() : Node("battery_node"), batteryLevel(false)
    {
        // Create a timer to periodically update the battery level
        timer = this->create_wall_timer(
            std::chrono::milliseconds(3400),
            std::bind(&BatteryNode::updateBatteryLevel, this));

        // Start a separate thread for service calls
        service_thread = std::thread(&BatteryNode::callBatteryStatus, this);
    }

    ~BatteryNode()
    {
        // Clean up the service thread before exiting
        service_thread.join();
    }

    void updateBatteryLevel()
    {
        batteryLevel = (batteryLevel + 1) % 4;
        // Notify the condition variable that battery level has changed
        cv.notify_one();
    }

    void callBatteryStatus()
    {
        auto client = this->create_client<my_robot_interfaces::srv::SetLed>("set_led");

        while (rclcpp::ok())
        {
            // Wait for the condition variable to be notified (battery level change)
            std::unique_lock<std::mutex> lock(mutex);
            cv.wait(lock);

            if (client->wait_for_service(std::chrono::seconds(1)))
            {
                // Create a shared pointer for the request
                auto request = std::make_shared<my_robot_interfaces::srv::SetLed::Request>();
                if (batteryLevel == 0)
                {
                    request->state = false;
                }
                else
                {
                    request->state = true;
                }
                request->lednumber = batteryLevel;

                auto future = client->async_send_request(request); // Sends request to the server

                // Blocks until a response from the future, can throw an exception
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
            else
            {
                RCLCPP_WARN(this->get_logger(), "Waiting for the server to be up...");
            }
        }
    }

private:
    rclcpp::TimerBase::SharedPtr timer;
    int batteryLevel; // 0 is empty, 3 is full

    std::mutex mutex;
    std::condition_variable cv;
    std::thread service_thread;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BatteryNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
