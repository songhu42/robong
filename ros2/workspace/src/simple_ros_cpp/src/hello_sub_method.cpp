#include "simple_ros_cpp/hello_sub.hpp"


MySubscriber::MySubscriber() : Node("hello_sub") {
    _sub = this->create_subscription<std_msgs::msg::String>("message", 10, std::bind(&MySubscriber::sub_callback, this, std::placeholders::_1)); 
}

void MySubscriber::sub_callback(const std_msgs::msg::String::SharedPtr msg) {
    // cout << "Subscriber get message! >> " << msg->data << endl; 
    RCLCPP_INFO(this->get_logger(),msg->data.c_str() ); 
}        
