#include <iostream>
#include <chrono> 
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"

using namespace std; 
void printHello(); 

typedef shared_ptr<std_srvs::srv::SetBool::Request> Req; 
typedef shared_ptr<std_srvs::srv::SetBool::Response> Res; 

class ServiceServer : public rclcpp::Node {
    public:
        ServiceServer() : Node("ServiceServer") {
            _server = create_service<std_srvs::srv::SetBool>(
                "setBoolService", 
                std::bind(&ServiceServer::service_callback, this, std::placeholders::_1, std::placeholders::_2)); 
                
            
            // _twist_pub_timer = create_wall_timer(1s, std::bind(&ServiceServer::twist_pub, this)); 
            // _update_timer = create_wall_timer(17ms, std::bind(&ServiceServer::update, this)); 
        }
    private:
        int _count; 
        bool isTrue = false; 
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr _server; 
        rclcpp::TimerBase::SharedPtr _twist_pub_timer; 
        rclcpp::TimerBase::SharedPtr _update_timer; 
        
        void service_callback(const Req request, Res response) {
            RCLCPP_INFO(get_logger(), "setBoolService received " + request->data); 
            
            isTrue = request->data; 

            if (request->data ) {
                response->success = true; 
                response->message = "True response"; 
            } else {
                response->success = false; 
                response->message = "False response"; 
            }
            
        }
}; 

int main()
{
    rclcpp::init(0, nullptr);
    auto node = std::make_shared<ServiceServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}