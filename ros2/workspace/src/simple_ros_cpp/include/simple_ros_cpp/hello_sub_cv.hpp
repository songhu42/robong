#ifndef __HELLO_SUB_CV__
#define __HELLO_SUB_CV__

#include <iostream>
#include <chrono> 
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "opencv2/opencv.hpp"

using namespace std; 
using namespace std::chrono_literals; 
using namespace cv; 

class MySubscriber: public rclcpp::Node {
    public:
        MySubscriber() ; 
        
    private:
        int _cnt = 0; 
        Mat _img; 
        void sub_callback(const std_msgs::msg::String::SharedPtr msg); 
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _sub;  
}; 


#endif