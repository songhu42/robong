#include <iostream>
#include <chrono> 
#include "rclcpp/rclcpp.hpp"

using namespace std; 
void printHello(); 

class SimpleParam : public rclcpp::Node {
    public:
        SimpleParam() : Node("SimpleParam") {
            _my_param = declare_parameter<string>("my_param", "내가 만든 param"); 
            _my_param2 = declare_parameter<string>("my_param2", "내가 만든 param2"); 
            _set_param_handle = add_on_set_parameters_callback(bind(&SimpleParam::param_callback, this, placeholders::_1)); 

            RCLCPP_INFO(get_logger(), "service is available!! "); 

             _update_timer = create_wall_timer(1s, std::bind(&SimpleParam::update, this)); 
        }
    private:
        int _count; 
        string _my_param; 
        string _my_param2; 
        rclcpp::TimerBase::SharedPtr _update_timer; 
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr _set_param_handle; 
        void update() {
            RCLCPP_INFO(get_logger(), "my_param : %s", _my_param.c_str()); 
            RCLCPP_INFO(get_logger(), "my_param2 : %s", _my_param2.c_str()); 

        }
        rcl_interfaces::msg::SetParametersResult param_callback(const vector<rclcpp::Parameter> &params) {
            for( auto param  : params ) {
                if(param.get_name() == "my_param") {
                    _my_param = param.as_string(); 
                } else if(param.get_name() == "my_param2") {
                    _my_param2 = param.as_string(); 
                }
            }
            rcl_interfaces::msg::SetParametersResult result; 
            result.successful = true; 
            return result; 
        }
}; 

int main()
{
    rclcpp::init(0, nullptr);
    auto node = std::make_shared<SimpleParam>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}