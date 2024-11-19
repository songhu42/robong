#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>

using namespace std;
using namespace std::chrono_literals;

class PublishMap : public rclcpp::Node
{
public:
    explicit PublishMap()
        : Node("publish_map"), _count(0)
    {
        _pub = create_publisher<nav_msgs::msg::OccupancyGrid>("myMap", 10);
        _timer = create_wall_timer(1s, std::bind(&PublishMap::pub_callback, this));
        init_pub(); 
    }

private:
    unsigned int _count;
    nav_msgs::msg::OccupancyGrid msg = nav_msgs::msg::OccupancyGrid();
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr _pub;
    rclcpp::TimerBase::SharedPtr _timer;
    void init_pub() {

        msg.header.frame_id = "world";
        msg.header.stamp = get_clock()->now();

        // map info
        msg.info.resolution = 0.1f;
        msg.info.width = 100;
        msg.info.height = 100;

        msg.info.origin.position.x = -(msg.info.width * msg.info.resolution) / 2;
        msg.info.origin.position.y = -(msg.info.height * msg.info.resolution) / 2;
        msg.info.origin.position.z = 0;
        msg.info.origin.orientation.x = 0;
        msg.info.origin.orientation.y = 0;
        msg.info.origin.orientation.z = 0;
        msg.info.origin.orientation.w = 1;

        msg.data.resize(msg.info.width * msg.info.height);
        for(auto &elm : msg.data)  elm = -1; // -1 : 탐사 안됨, 0 : 못감, 1~100 : 갈 수 있음. 

        _pub->publish(msg);
    }

    void pub_callback()
    {
        // 1 block씩 block 처리 
        for(int i=0; i<10; i++ )  {
            msg.data[msg.info.width*_count + i] = 0; 
            for(int j=10; j<100; j++ ) msg.data[msg.info.width*_count + j] = 1; 
        }
        _count++; 
        if(_count > 9 ) _count = 0; 

        _pub->publish(msg);
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PublishMap>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
