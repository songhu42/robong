#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include <chrono>
#include <string>
#include "inttypes.h"

using namespace std;
using namespace std::chrono_literals;

class Producer_node : public rclcpp::Node
{
public:
    Producer_node(const string &name, const string &output)
        : Node(name, rclcpp::NodeOptions().use_intra_process_comms(true)), _count(0)
    {
        _pub = create_publisher<std_msgs::msg::Int32>(output, 10);
        // decltype : type 추론 => remove_pointer 제거 .. 아래와 같은 의미이다. 
        // weak_ptr<remove_pointer<decltype(_pub.get())>::type> captured_pub = _pub; 
        rclcpp::Publisher<std_msgs::msg::Int32>::WeakPtr captured_pub = _pub;

        auto pub_callback = [captured_pub]()
        {
            // lock the pub object for thread safe
            auto pub_ptr = captured_pub.lock(); 
            if( !pub_ptr ) return; 

            std_msgs::msg::Int32::UniquePtr msg(new std_msgs::msg::Int32()); 
            static int32_t count = 0; 
            msg->data = count++; 

            // 출력 졸라 어렵네.. ㅋㅋㅋ 
            printf("Pub msg : %d address : 0x%" PRIXPTR "\n", msg->data, reinterpret_cast<uintptr_t>(msg.get()));

            // 메모리 절약을 위해 주소값을 이동한다. 주소값을 하나의 Node에서만 Access 가능함. 
            pub_ptr->publish(move(msg)); 
            // RCLCPP_INFO(get_logger(), msg.data.c_str()); 
        }; 
        
        _timer = create_wall_timer(1s, pub_callback);
    }

private:
    size_t _count;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr _pub;
    rclcpp::TimerBase::SharedPtr _timer; 
    
};


class Consumer_node : public rclcpp::Node
{
public:
    Consumer_node(const string &name, const string &output)
        : Node(name, rclcpp::NodeOptions().use_intra_process_comms(true)), _count(0)
    {
        _sub = create_subscription<std_msgs::msg::Int32>(output, 10, 
        
        [](std_msgs::msg::Int32::UniquePtr msg){
            printf("message received %d address 0x%" PRIXPTR "\n", msg->data, reinterpret_cast<uintptr_t>(msg.get())); 
        });

        rclcpp::Subscription<std_msgs::msg::Int32>::WeakPtr captured_pub = _sub;
    }

private:
    size_t _count;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr _sub;
    rclcpp::TimerBase::SharedPtr _timer; 
    
};


int main(int argc, char* argv[])
{
    // pipeline buffer memory allocation
    setvbuf(stdout, NULL, _IONBF, BUFSIZ); 

    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor excutor; 
    auto producer_node = std::make_shared<Producer_node>("producer", "number");
    auto consumer_node = std::make_shared<Consumer_node>("producer", "number");
    excutor.add_node(producer_node); 
    excutor.add_node(consumer_node); 
    excutor.spin();

    rclcpp::shutdown();
    return 0;
}