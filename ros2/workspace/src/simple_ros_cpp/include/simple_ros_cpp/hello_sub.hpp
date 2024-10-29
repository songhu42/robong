#ifndef __HELLO_SUB__
#define __HELLO_SUB__

#include <iostream>
#include <chrono> 
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std; 
using namespace std::chrono_literals; 

class MySubscriber: public rclcpp::Node {
    public:
        MySubscriber() ; 
        
    private:
        int _cnt = 0; 
        void sub_callback(const std_msgs::msg::String::SharedPtr msg); 
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _sub;  
}; 


#endif