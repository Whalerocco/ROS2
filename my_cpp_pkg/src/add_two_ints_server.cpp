#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class AddTwoIntsServerNode : public rclcpp::Node
{
public:
    AddTwoIntsServerNode() : Node("add_two_ints_server")
    {
        //Two placeholders are used below since the callback has two input arguments.
        server = this->create_service<example_interfaces::srv::AddTwoInts>("add_two_ints",
                std::bind(&AddTwoIntsServerNode::callbackAddTwoInts, this, _1,_2));
    
        RCLCPP_INFO(this->get_logger(), "Service has been started.");
    }

private:
    // callback, only called from inside class. Called when server is called by a client
    void callbackAddTwoInts(const example_interfaces::srv::AddTwoInts::Request::SharedPtr request,
                            const example_interfaces::srv::AddTwoInts::Response::SharedPtr response)
    {
        // Response automatically sent back to client when the object is filled
        response->sum = request->a + request->b;
        RCLCPP_INFO(this->get_logger(), "%d + %d = %d", request->a, request->b, response->sum);
    }
    rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr server;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AddTwoIntsServerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}