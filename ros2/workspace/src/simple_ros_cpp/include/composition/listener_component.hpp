#ifndef COMPOSITION__LISTENER_COMPONENT_HPP_
#define COMPOSITION__LISTENER_COMPONENT_HPP_

#include "composition/visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace composition
{

    class Listener : public rclcpp::Node
    {
    public:
        COMPOSITION_PUBLIC
        explicit Listener(const rclcpp::NodeOptions &options);

    protected:
        void on_subscription(std_msgs::msg::String::ConstSharedPtr msg);

    private:
        size_t _count;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _sub;
    };
}

#endif // COMPOSITION__LISTENER_COMPONENT_HPP_