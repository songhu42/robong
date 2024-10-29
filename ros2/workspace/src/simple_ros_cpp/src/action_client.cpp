#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "user_interface/action/fibonacci.hpp"
#include <chrono>
#include <iostream>
#include <thread>

using namespace std;
using namespace std::placeholders;
using namespace std::chrono_literals;

typedef user_interface::action::Fibonacci Fibonacci;
typedef rclcpp_action::ClientGoalHandle<Fibonacci> GoalHandleFibonacci;


class ActionClient : public rclcpp::Node
{
public:
    ActionClient()
        : Node("action_client")
    {
        _action_client = rclcpp_action::create_client<Fibonacci>(
            this, "fibonacci");
    }

    void send_goal(char* step) {
        while( !_action_client->wait_for_action_server(1s) ) {
            retry_cnt++; 
            RCLCPP_INFO(get_logger(), "can not connect server retry : %d", retry_cnt);
        }

        Fibonacci::Goal goal_msg = Fibonacci::Goal(); 
        auto options = rclcpp_action::Client<Fibonacci>::SendGoalOptions(); 
        options.goal_response_callback = bind(&ActionClient::goal_response_callback, this, _1); 
        options.feedback_callback = bind(&ActionClient::feedback_callback, this, _1, _2); 
        options.result_callback = bind(&ActionClient::result_callback, this, _1); 

        goal_msg.req_step = atoi(step); 
        _action_client->async_send_goal(goal_msg, options); 
    }

private:
    int retry_cnt = 0; 
    rclcpp_action::Client<Fibonacci>::SharedPtr _action_client;

    void goal_response_callback(
        const GoalHandleFibonacci::SharedPtr &goal_handle)
    {
        RCLCPP_INFO(get_logger(), "goal_response_callback"); 
        if( goal_handle ) {
            RCLCPP_INFO(get_logger(), "goal_response_callback goal_handle exist!!"); 
        }
    }

    void feedback_callback(
        const GoalHandleFibonacci::SharedPtr &goal_handle, 
        const shared_ptr<const Fibonacci::Feedback> feedback)
    {
        RCLCPP_INFO(get_logger(), "feedback_callback"); 
        goal_handle->get_goal_id(); 
        for(int seq : feedback->feedback_seq) {
            RCLCPP_INFO(get_logger(), "feedback_seq : %d", seq); 
        }
    }
    void result_callback(const GoalHandleFibonacci::WrappedResult &result)
    {
        RCLCPP_INFO(get_logger(), "result_callback"); 

        if( result.code == rclcpp_action::ResultCode::SUCCEEDED ) {
            
            for (int seq : result.result->result_seq) {
                RCLCPP_INFO(get_logger(), "seq : %d", seq);  
            } 
        } else if( result.code == rclcpp_action::ResultCode::CANCELED ) {
            RCLCPP_INFO(get_logger(), "result_callback => CANCELED"); 
        } 
        
    }

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ActionClient>();
    node->send_goal(argv[1]); 

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}