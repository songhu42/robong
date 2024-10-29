#include "simple_ros_cpp/hello_pub_class.hpp"


MyPublisher::MyPublisher() : Node("hello"), _cnt(0) {
    _pub = this->create_publisher<std_msgs::msg::String>("message", 10); 
    _timer = this->create_wall_timer(1s, std::bind(&MyPublisher::printHello, this)); 
}

void MyPublisher::printHello() {
    auto msg = std_msgs::msg::String();
    msg.data = "Hello Class " + to_string(_cnt); 
    _pub->publish(msg); 
    cout << "Hello Class! >> " << _cnt << endl; 
    _cnt++; 
}        
