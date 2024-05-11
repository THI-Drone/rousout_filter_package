#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/Heartbeat.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher(char* id)
  : Node("minimal_publisher")
  {
    this->id = id;
    publisher_ = this->create_publisher<interfaces::msg::Heartbeat>("heartbeat", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = interfaces::msg::Heartbeat();
    message.sender_id = id;
    message.active = false;
    message.tick = ++tick_;
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<interfaces::msg::Heartbeat>::SharedPtr publisher_;
  uint32_t tick_ = 0;
  char* id;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>(argv[1]));
  rclcpp::shutdown();
  return 0;
}
