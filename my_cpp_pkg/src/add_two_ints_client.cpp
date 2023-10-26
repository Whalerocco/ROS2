#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

class AddTwoIntsClientNode : public rclcpp::Node  
{
public:
    AddTwoIntsClientNode() : Node("add_two_ints_client")  
    {
        //callAddTwoIntsService(1,5);
        //The callback needs to be called in another threadd, otherwise this node will get stuck
        //thread1  = std::thread(std::bind(&AddTwoIntsClientNode::callAddTwoIntsService, this, 1, 4));
        
        //To send multiple calls to the server, a thread pool can be implemented.
        threads.push_back(std::thread(std::bind(&AddTwoIntsClientNode::callAddTwoIntsService, this, 1, 4)));
        threads.push_back(std::thread(std::bind(&AddTwoIntsClientNode::callAddTwoIntsService, this, -35, 7)));
        threads.push_back(std::thread(std::bind(&AddTwoIntsClientNode::callAddTwoIntsService, this, -41, 4)));
        
    }


    void callAddTwoIntsService(int a, int b)
    {
        auto client = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
        while(!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for the server to be up...");
        }

        //create a shared pointer
        auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
        request->a = a;
        request->b = b;

        auto future = client->async_send_request(request); //Sends request to the server

        //Blocks until a response from the future, can throw eerror  wheen exception
        try
        {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "%d + %d = %d", a, b, response->sum);
        }
        catch(const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed");
        }

        
    }
private:
    //Thread object needs to be a member of the class
    std::thread thread1;
    std::vector<std::thread> threads;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AddTwoIntsClientNode>();  
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}