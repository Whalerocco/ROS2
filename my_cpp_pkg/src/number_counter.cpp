#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"
#include "example_interfaces/srv/set_bool.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

//Subscribes to a number topic
//Counts the numbers
//Publishes the sum to a topic
class NumberCounterNode : public rclcpp::Node  
{
public:
    NumberCounterNode() : Node("counter_node"), counter(0)
    {
        //Init publisher
        counter_publisher = this->create_publisher<example_interfaces::msg::Int64>("countTopic", 10);
        //Init subscriber
        nbr_subscriber = this->create_subscription<example_interfaces::msg::Int64>("numberTopic", 10, std::bind(&NumberCounterNode::callbackCounter, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Counter node started.");

        //Server
        //Two placeholders are used below since the callback has two input arguments.
        server = this->create_service<example_interfaces::srv::SetBool>("reset_counter",
                std::bind(&NumberCounterNode::callbackResetCounter, this, _1,_2));
    
        RCLCPP_INFO(this->get_logger(), "Service has been started.");
    }

   

private:
    void callbackCounter(const example_interfaces::msg::Int64::SharedPtr inMsg)
    {// callback, only called from inside class. Called when server is called by a client
        counter += inMsg->data;
        auto outMsg = example_interfaces::msg::Int64();
        outMsg.data = counter;
        counter_publisher->publish(outMsg);
        RCLCPP_INFO(this->get_logger(), "data %i", counter);
    }

    // callback, only called from inside class. Called when server is called by a client
    void callbackResetCounter(const example_interfaces::srv::SetBool::Request::SharedPtr request,
                            const example_interfaces::srv::SetBool::Response::SharedPtr response)
    {
        // Response automatically sent back to client when the object is filled
        if (request->data == true)
        {
            counter = 0;
            response->success = true;
            response->message = "Counter reset to 0.";
        }
        else
        {
            response->success = false;
            response->message = "Counter not reset.";
        }
    }
    rclcpp::Service<example_interfaces::srv::SetBool>::SharedPtr server;

    int counter;
    //declare publisher
    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr counter_publisher;
    //declare subscriber
    rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr nbr_subscriber;
    
    
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberCounterNode>();  
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}