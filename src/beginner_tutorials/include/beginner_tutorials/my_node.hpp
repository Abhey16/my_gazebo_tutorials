#pragma once

#include <example_interfaces/msg/detail/string__struct.hpp>
#include <example_interfaces/msg/string.hpp>
#include <example_interfaces/srv/detail/set_bool__struct.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/timer.hpp>

// Declaring MyNode class
class MyNode : public rclcpp::Node {
 public:
  // Constructor
  MyNode();

 private:
  // Callback function for publishing custom string
  void publishCallback();
  void serverCallback(const example_interfaces::srv::SetBool::Request::SharedPtr request, 
                      const example_interfaces::srv::SetBool::Response::SharedPtr response);

  // Declaring attributes
  rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<example_interfaces::srv::SetBool>::SharedPtr server_;
  std::string base_message_;
};