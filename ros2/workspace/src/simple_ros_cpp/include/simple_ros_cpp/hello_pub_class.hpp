#ifndef __HELLO_PUB_CLASS__
#define __HELLO_PUB_CLASS__

#include <iostream>
#include <chrono> 
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std; 
using namespace std::chrono_literals; 

class MyPublisher : public rclcpp::Node {
    public:
        MyPublisher() ; 
        void printHello(); 
    private:
        int _cnt = 0; 

        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _pub; 
        rclcpp::TimerBase::SharedPtr _timer; 
}; 


#endif