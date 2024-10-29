#include "simple_ros_cpp/hello_pub_class.hpp"

int main() {
    rclcpp::init(0, nullptr); 
    auto node = make_shared<MyPublisher>(); 

    rclcpp::spin(node); 
    rclcpp::shutdown(); 

    cout << "Terminated!" << endl; 
    return 0;
}
