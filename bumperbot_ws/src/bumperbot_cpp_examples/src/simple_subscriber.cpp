#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <string>

#include <chrono>
using std::placeholders::_1;

class SimpleSubscriber : public rclcpp::Node
{
private:
    //const std::string node_name  = "simple_subscriber";
    const std::string topic_name = "chatter";
    const int que_size = 10;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;

    void msgCallback(const std_msgs::msg::String &msg) const
    {
        RCLCPP_INFO_STREAM(get_logger(), "I heard:" << msg.data.c_str());
    }

public:
    SimpleSubscriber() : Node("simple_subscriber")
    {
        sub_ = create_subscription<std_msgs::msg::String>(topic_name, que_size, std::bind(&SimpleSubscriber::msgCallback, this, _1));
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}