#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include <chrono>
#include <string>
#include "inttypes.h"

using namespace std;
using namespace std::chrono_literals;

class Cyclic_node : public rclcpp::Node
{
public:
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr _pub;

    Cyclic_node(const string &name, const std::string &in, const std::string &out)
        : Node(name, rclcpp::NodeOptions().use_intra_process_comms(true)) 
    {
        _pub = create_publisher<std_msgs::msg::Int32>(out, 10);
        // decltype : type 추론 => remove_pointer 제거 .. 아래와 같은 의미이다. 
        // weak_ptr<remove_pointer<decltype(_pub.get())>::type> captured_pub = _pub; 
        rclcpp::Publisher<std_msgs::msg::Int32>::WeakPtr captured_pub = _pub;
        
        _sub = create_subscription<std_msgs::msg::Int32>(in, 10, 
        [this, captured_pub](std_msgs::msg::Int32::UniquePtr msg){
                        // lock the pub object for thread safe
            auto pub_ptr = captured_pub.lock(); 
            if( !pub_ptr ) return; 

            if (!rclcpp::sleep_for(1s))
            {
                return;
            }

            // printf("Pub msg : %d address : 0x%" PRIXPTR "\n", msg->data, reinterpret_cast<uintptr_t>(msg.get()));
            RCLCPP_INFO(get_logger(), "Receive msg value %d address 0x%" PRIXPTR, msg->data,
                            reinterpret_cast<std::uintptr_t>(msg.get()));
            // 메모리 절약을 위해 주소값을 이동한다. 주소값을 하나의 Node에서만 Access 가능함. 
            msg->data ++; 
            pub_ptr->publish(move(msg)); 
        });

    }

private:
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr _sub;
};

int main(int argc, char* argv[])
{
    // pipeline buffer memory allocation
    setvbuf(stdout, NULL, _IONBF, BUFSIZ); 

    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor excutor; 
    auto pipe_node1 = std::make_shared<Cyclic_node>("pipe_node1", "topic1", "topic2");
    auto pipe_node2 = std::make_shared<Cyclic_node>("pipe_node2", "topic2", "topic1");
    excutor.add_node(pipe_node1); 
    excutor.add_node(pipe_node2); 

    rclcpp::sleep_for(1s); 
    
    std_msgs::msg::Int32::UniquePtr msg(new std_msgs::msg::Int32()); 
    msg->data = 42; 
    printf("Pub msg : %d address : 0x%" PRIXPTR "\n", msg->data, reinterpret_cast<uintptr_t>(msg.get()));
    pipe_node1->_pub->publish(move(msg)); 

    excutor.spin();

    rclcpp::shutdown();
    return 0;
}