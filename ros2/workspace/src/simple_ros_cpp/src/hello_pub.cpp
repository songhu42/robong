#include <iostream>
#include <chrono> 
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


using namespace std; 
void printHello(); 

int main() {
    int cnt = 0; 
    rclcpp::init(0, nullptr); 
    auto node = make_shared<rclcpp::Node>("hello"); 
    auto pub = node->create_publisher<std_msgs::msg::String>("message", 10); 
    // auto timer = node->create_wall_timer(1s, printHello); 
    auto timer = node->create_wall_timer(1s, [&cnt, pub]() {
        auto msg = std_msgs::msg::String(); 
        msg.data = "Hello msg " + to_string(cnt); 
        pub->publish(msg); 
        cout << "Hello pub! >> " << cnt << endl; 
        cnt++; 

    }
    ); 
    rclcpp::spin(node); 
    rclcpp::shutdown(); 

    cout << "Terminated!" << endl; 
    return 0;
}
void printHello() {
    static int cnt = 0; 
    cout << "Hello rclcpp! >> " << cnt << endl; 
    cnt++; 
}