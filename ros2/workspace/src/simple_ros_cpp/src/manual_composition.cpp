#include <memory>

#include "composition/talker_component.hpp"
#include "composition/listener_component.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char* argv[]) {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ); 

    rclcpp::init(argc, argv); 

    rclcpp::executors::SingleThreadedExecutor exec; 
    rclcpp::NodeOptions opts; 

    auto talker = std::make_shared<composition::Talker>(opts); 
    auto listener = std::make_shared<composition::Listener>(opts); 

    exec.add_node(talker); 
    exec.add_node(listener); 

    exec.spin(); 
    rclcpp::shutdown(); 
    
    return 0; 
}