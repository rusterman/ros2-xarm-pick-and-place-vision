#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class HelloSubscriber : public rclcpp::Node
{
public:
  HelloSubscriber() : Node("hello_cpp_subscriber")
  {
    sub_ = this->create_subscription<std_msgs::msg::String>(
      "hello_topic_cpp", 10,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received: %s", msg->data.c_str());
      });
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HelloSubscriber>());
  rclcpp::shutdown();
  return 0;
}
