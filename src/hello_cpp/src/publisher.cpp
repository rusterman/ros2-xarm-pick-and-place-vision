#include <chrono>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class HelloPublisher : public rclcpp::Node
{
public:
  HelloPublisher() : Node("hello_cpp_publisher"), count_(0)
  {
    pub_ = this->create_publisher<std_msgs::msg::String>("hello_topic_cpp", 10);
    timer_ = this->create_wall_timer(1s, [this]() {
      auto msg = std_msgs::msg::String();
      msg.data = "Hello from C++ #" + std::to_string(count_++);
      RCLCPP_INFO(this->get_logger(), "Publishing: %s", msg.data.c_str());
      pub_->publish(msg);
    });
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HelloPublisher>());
  rclcpp::shutdown();
  return 0;
}
