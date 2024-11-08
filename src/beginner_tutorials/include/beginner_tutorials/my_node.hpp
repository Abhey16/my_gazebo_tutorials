#pragma once

#include <example_interfaces/msg/detail/string__struct.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/msg/string.hpp>
#include <rclcpp/timer.hpp>

class MyNode : public rclcpp::Node
{
public:
    // Constructor
    MyNode();
    
private:
    // Callback function for publishing custom string
    void publishCallback();

    // Declaring attributes
    rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

};