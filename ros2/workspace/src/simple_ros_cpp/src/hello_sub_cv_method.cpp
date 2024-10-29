#include "simple_ros_cpp/hello_sub_cv.hpp"


MySubscriber::MySubscriber() : Node("hello_sub"), _img(500, 500, CV_8UC1, Scalar(255)) {
    _sub = this->create_subscription<std_msgs::msg::String>("message", 10, std::bind(&MySubscriber::sub_callback, this, std::placeholders::_1)); 
}

void MySubscriber::sub_callback(const std_msgs::msg::String::SharedPtr msg) {
    // cout << "Subscriber get message! >> " << msg->data << endl; 
    _img = Scalar(255);
    RCLCPP_INFO(this->get_logger(),msg->data.c_str() ); 
    putText(_img, msg->data.c_str(), Point(50, 50), FONT_HERSHEY_PLAIN, 1, Scalar(0), 2); 
    imshow("img", _img); 
    waitKey(30); 
}        
