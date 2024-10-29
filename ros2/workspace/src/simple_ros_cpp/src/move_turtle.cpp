#include <iostream>
#include <chrono> 
#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/color.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std; 
void printHello(); 

class MoveTurtle : public rclcpp::Node {
    public:
        MoveTurtle() : Node("move_turtle") {
            auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
                .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE); 
            _pose_sub = create_subscription<turtlesim::msg::Pose>(
                "turtle1/pose", qos_profile, 
                std::bind(&MoveTurtle::pose_sub_callback, this, std::placeholders::_1)); 
            _color_sub = create_subscription<turtlesim::msg::Color>(
                "turtle1/color", qos_profile,  
                std::bind(&MoveTurtle::color_sub_callback, this, std::placeholders::_1)); 

            _pub = create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", qos_profile);
            _twist_pub_timer = create_wall_timer(1s, std::bind(&MoveTurtle::twist_pub, this)); 
            _update_timer = create_wall_timer(17ms, std::bind(&MoveTurtle::update, this)); 
        }
    private:
        int _count; 
        turtlesim::msg::Color _color; 
        turtlesim::msg::Pose _pose; 
        geometry_msgs::msg::Twist _twist; 

        rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr _pose_sub; 
        rclcpp::Subscription<turtlesim::msg::Color>::SharedPtr _color_sub; 
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _pub; 
        rclcpp::TimerBase::SharedPtr _twist_pub_timer; 
        rclcpp::TimerBase::SharedPtr _update_timer; 
        
        void pose_sub_callback(const turtlesim::msg::Pose::SharedPtr msg) {
            _pose = *msg; 
        }
        void color_sub_callback(const turtlesim::msg::Color::SharedPtr msg) {
            _color = *msg; 
        }

        void twist_pub() {
            _pub->publish(_twist); 
        }

        void update() {
            _twist.linear.x += 0.004; 
            _twist.angular.z = 1.0; 
        }
}; 

int main()
{
    rclcpp::init(0, nullptr);
    auto node = std::make_shared<MoveTurtle>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}