#pragma once

#include <example_interfaces/msg/detail/string__struct.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/msg/string.hpp>
#include <rclcpp/subscription.hpp>


class SubscriberNode : public rclcpp::Node
{
public:
    SubscriberNode();

private:
    void subscriberCallback(example_interfaces::msg::String::SharedPtr msg);

    rclcpp::Subscription<example_interfaces::msg::String>::SharedPtr subscriber_;

};