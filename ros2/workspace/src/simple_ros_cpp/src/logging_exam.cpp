#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>

using namespace std;
using namespace std::chrono_literals;

bool is_divisor_of_twelve(size_t val, rclcpp::Logger logger);

class LoggerUsage : public rclcpp::Node
{
public:
    LoggerUsage()
        : Node("logger_usage_demo"), _count(0)
    {
        _pub = create_publisher<std_msgs::msg::String>("message", 10);
        _timer = create_wall_timer(1s, std::bind(&LoggerUsage::pub_callback, this));

        debug_function_to_evaluate = std::bind(is_divisor_of_twelve, std::cref(_count), get_logger());
        
        // lamda function using this reference .. 
        auto on_one_shot_timer = [this]() -> void
        {
            // 한번 실행 후 타이머 중지로 한번만 실행되게 함. 
            _one_shot_timer->cancel();

            // logger level 세팅 ..
            RCLCPP_INFO(get_logger(), "Setting serverity Debug");
            auto ret = rcutils_logging_set_logger_level(
                get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
            if (ret != RCUTILS_RET_OK)
            {
                RCLCPP_ERROR(get_logger(), "Error setting severity %s", rcutils_get_error_string().str);
            }
        };

        _one_shot_timer = create_wall_timer(1s, on_one_shot_timer);
    }

private:
    size_t _count;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _pub;
    rclcpp::TimerBase::SharedPtr _timer, _one_shot_timer;
    std::function<bool()> debug_function_to_evaluate;
    void pub_callback()
    {
        RCLCPP_INFO_ONCE(get_logger(), "Timer callback called(once)");
        auto msg = std_msgs::msg::String();
        msg.data = "Current count : " + std::to_string(_count);
        RCLCPP_INFO(get_logger(), msg.data.c_str());
        _pub->publish(msg);
    }
};

bool is_divisor_of_twelve(size_t val, rclcpp::Logger logger)
{
    if ( val % 12 == 0 ) {
        return true;
    }
    return false; 
}

int main()
{
    rclcpp::init(0, nullptr);
    auto node = std::make_shared<LoggerUsage>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}