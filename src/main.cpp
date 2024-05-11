#include "main.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.enable_rosout(false);
    rclcpp::Node::SharedPtr filter_node = std::make_shared<rclcpp::Node>("rosout_filter", node_options);
    const std::string logger_regex = filter_node->declare_parameter<std::string>("LOGGER_REGEX", "");
    bool regex_set;
    std::regex regex;
    if(logger_regex.empty()) {
        regex_set = false;
    } else {
        regex = std::regex(logger_regex);
        regex_set = true;
    }
    rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr rosout_subscriber = filter_node->create_subscription<rcl_interfaces::msg::Log>("/rosout", 10, [&regex_set, &regex, filter_node](const rcl_interfaces::msg::Log &log) -> void{
        if(regex_set) {
            if(!std::regex_match(log.name, regex)) {
                return;
            }
        }
        switch(log.level) {
            case rcl_interfaces::msg::Log::DEBUG:
                RCLCPP_DEBUG(filter_node->get_logger(), "[%d:%d] %s : %s : %d | %s", log.stamp.sec, log.stamp.nanosec, log.file.c_str(), log.function.c_str(), log.line, log.msg.c_str());
                break;
            case rcl_interfaces::msg::Log::INFO:
                RCLCPP_INFO(filter_node->get_logger(), "[%d:%d] %s : %s : %d | %s", log.stamp.sec, log.stamp.nanosec, log.file.c_str(), log.function.c_str(), log.line, log.msg.c_str());
                break;
            case rcl_interfaces::msg::Log::WARN:
                RCLCPP_WARN(filter_node->get_logger(), "[%d:%d] %s : %s : %d | %s", log.stamp.sec, log.stamp.nanosec, log.file.c_str(), log.function.c_str(), log.line, log.msg.c_str());
                break;
            case rcl_interfaces::msg::Log::ERROR:
                RCLCPP_ERROR(filter_node->get_logger(), "[%d:%d] %s : %s : %d | %s", log.stamp.sec, log.stamp.nanosec, log.file.c_str(), log.function.c_str(), log.line, log.msg.c_str());
                break;
            case rcl_interfaces::msg::Log::FATAL:
                RCLCPP_FATAL(filter_node->get_logger(), "[%d:%d] %s : %s : %d | %s", log.stamp.sec, log.stamp.nanosec, log.file.c_str(), log.function.c_str(), log.line, log.msg.c_str());
                break;
        }
    });
    rclcpp::spin(filter_node);
    rclcpp::shutdown();
    return 0;
}
