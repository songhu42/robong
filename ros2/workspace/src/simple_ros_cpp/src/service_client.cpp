#include <iostream>
#include <chrono> 
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"

using namespace std; 
void printHello(); 

typedef shared_ptr<std_srvs::srv::SetBool::Request> Req; 
typedef shared_ptr<std_srvs::srv::SetBool::Response> Res; 

class ServiceClient : public rclcpp::Node {
    public:
        ServiceClient() : Node("ServiceClient") {
            _client = create_client<std_srvs::srv::SetBool>("setBoolService"); 
            while(!_client->wait_for_service(1s) ) {
                RCLCPP_INFO(get_logger(), "service is not available!! "); 
            }    
            _request = std::make_shared<std_srvs::srv::SetBool::Request>(); 
            RCLCPP_INFO(get_logger(), "service is available!! "); 
            _request->data = isTrue; 
            send_request(_request); 

            // _twist_pub_timer = create_wall_timer(1s, std::bind(&ServiceClient::twist_pub, this)); 
             _update_timer = create_wall_timer(1s, std::bind(&ServiceClient::update, this)); 
        }
    private:
        int _count; 
        rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr _client; 
        rclcpp::TimerBase::SharedPtr _twist_pub_timer; 
        rclcpp::TimerBase::SharedPtr _update_timer; 
        bool isTrue = false; 
        Req _request;  

        void send_request(Req request) {
            RCLCPP_INFO(get_logger(), "send_request~~ "); 
            auto future = _client->async_send_request(request, bind(&ServiceClient::receive_callback, this, placeholders::_1)); 
        }

        void receive_callback(rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
            Res response = future.get(); 
            // string suc = (response->success)?"true":"false"; 
            RCLCPP_INFO(get_logger(), "result : %s", (response->success)?"true":"false"); 
            RCLCPP_INFO(get_logger(), "message : %s", response->message.c_str()); 
        }
        
        void update() {
            _request->data = isTrue = !isTrue; 
            send_request(_request); 
        }
}; 

int main()
{
    rclcpp::init(0, nullptr);
    auto node = std::make_shared<ServiceClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}