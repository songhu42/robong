#include "simple_ros_cpp/hello_sub.hpp"

int main() {
    rclcpp::init(0, nullptr); 
    auto node = make_shared<MySubscriber>(); 

    rclcpp::spin(node); 
    rclcpp::shutdown(); 

    cout << "Terminated!" << endl; 
    return 0;
}
