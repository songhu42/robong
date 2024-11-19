#include "composition/listener_component.hpp"
#include <chrono>

using namespace std::chrono_literals;
namespace composition
{
    Listener::Listener(const rclcpp::NodeOptions &options)
        : Node("listener", options), _count(0)
    {
        _sub = create_subscription<std_msgs::msg::String>("chatter", 10, bind(&Listener::on_subscription, this, std::placeholders::_1));
    }
    void Listener::on_subscription(std_msgs::msg::String::ConstSharedPtr msg)
    {
        RCLCPP_INFO(get_logger(), "I heard message %s", msg->data.c_str());
        std::flush(std::cout); 
    }
}
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(composition::Listener)