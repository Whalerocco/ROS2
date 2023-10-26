#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/msg/led_states.hpp"
#include "my_robot_interfaces/srv/set_led.hpp"

using namespace std;
using std::placeholders::_1;
using std::placeholders::_2;

//This node is a server for a service call from the BatteryNode client
//It's also a publisher to a led_panel_state topic
class LedPanelNode : public rclcpp::Node  
{
public:
    LedPanelNode() : Node("led_panel_server")  
    {
        this->declare_parameter("ledStat", std::vector<int64_t>{0,0,0});
        ledStatus = this->get_parameter("ledStat").as_integer_array();
        //int i = 0;
        /*for(int a : this->get_parameter("ledStatus").as_integer_array())
        {
            ledStatus[i] = a;
        }
        */
        /*ledStatus[0] = 0;
        ledStatus[1] = 0;
        ledStatus[2] = 0;
        */
        ledString = "[0 0 0]";
        buildLedString();

        //Init a publisher to publish to the 'led panel state' topic
        pub = this->create_publisher<my_robot_interfaces::msg::LedStates>("led_panel_state", 10);
        //Init a timer to publish the LED status every second
        timer = this->create_wall_timer(
            std::chrono::milliseconds(1000), 
            std::bind(&LedPanelNode::publishLedStatus, this));
            
        //Two placeholders are used below since the callback has two input arguments.
        server = this->create_service<my_robot_interfaces::srv::SetLed>("set_led",
                std::bind(&LedPanelNode::callbackUpdateLED, this, _1,_2));

        RCLCPP_INFO(this->get_logger(), "LED Panel node started.");
        
    }

private:
    //Publishes the led panel to a topic each second, also prints it
    void publishLedStatus()
    {
        auto msg = my_robot_interfaces::msg::LedStates();
        
        msg.message = ledString;
        pub->publish(msg);
        RCLCPP_INFO(this->get_logger(), "%s", ledString.c_str());
    }

    rclcpp::Publisher<my_robot_interfaces::msg::LedStates>::SharedPtr pub;
    rclcpp::TimerBase::SharedPtr timer;

    // Build the LED string
    void buildLedString()
    {
        std::cout << ledStatus[0] << " och " << ledStatus[1] << " och " << ledStatus[2] <<endl;
        for (size_t i = 0; i < 3; i++)
        {
            if( ledStatus.at(i) == 1) //Bättre att använda .at() ist för [] för vektorer eftersom .at() ger ett exception om out of bounds
            {
                ledString[2*i+1] = '*';
            }
            else
            {
                ledString[2*i+1] = '0';
            }
        }
    }

    //Run when client calls
    //updates LED status int array and sets message string
    void callbackUpdateLED(const my_robot_interfaces::srv::SetLed::Request::SharedPtr request,
                            const my_robot_interfaces::srv::SetLed::Response::SharedPtr response)
    {
        if( request->lednumber == 1 )
        {
            if( request->state )
            {
                ledStatus[0] = 1;
            }
            else
            {
                ledStatus[0] = 0;
            }
        }
        else if( request->lednumber == 2 )
        {
            if( request->state )
            {
                ledStatus[1] = 1;
            }
            else
            {
                ledStatus[1] = 0;
            }
        }    
        else if( request->lednumber == 3 )
        {
            if( request->state )
            {
                ledStatus[2] = 1;
            }
            else
            {
                ledStatus[2] = 0;
            }
        }
        else if( (request->lednumber == 0))
        {
            ledStatus[0] = 0;
            ledStatus[1] = 0;
            ledStatus[2] = 0;
        }

        buildLedString();
        
        //RCLCPP_INFO(this->get_logger(), "%s", ledString);

        // Response automatically sent back to client when the object is filled
        response->success = true;
    }

    rclcpp::Service<my_robot_interfaces::srv::SetLed>::SharedPtr server;
    
    std::vector<int64_t> ledStatus; //{0,0,1} means last lamp is on
    string ledString; //outputs a string representing the LED panel [0 0 0]

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LedPanelNode>();  
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}