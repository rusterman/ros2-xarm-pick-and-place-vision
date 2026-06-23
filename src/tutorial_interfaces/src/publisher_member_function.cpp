#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "tutorial_interfaces/msg/num.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node {
public:
    MinimalPublisher() : Node("minimal_publisher"), count_(0) {
        publisher_ = this->create_publisher<tutorial_interfaces::msg::Num>("topic", 10);
        timer_ = this->create_wall_timer(500ms, std::bind(&MinimalPublisher::timerCallback, this));
    }

private:
    size_t count_;
    rclcpp::Publisher<tutorial_interfaces::msg::Num>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    void timerCallback() {
        auto message = tutorial_interfaces::msg::Num();
        message.num = this->count_++;
        RCLCPP_INFO(this->get_logger(), "Publishing: '%ld'", message.num);
        publisher_->publish(message);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();

    return 0;
}