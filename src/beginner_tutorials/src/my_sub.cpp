// MIT License
//
// Copyright (c) 2024 Abhey Sharma

/**
 * @file my_sub.cpp
 * @brief Implementation file for the SubscriberNode class
 * @details Contains the implementation of the SubscriberNode class methods
 * including constructor, callback, and main function
 */

#include <memory>

#include "beginner_tutorials/my_sub.hpp"
#include <example_interfaces/msg/detail/string__struct.hpp>
#include <rclcpp/logging.hpp>

SubscriberNode::SubscriberNode() : Node("my_sub") {
  subscriber_ = this->create_subscription<example_interfaces::msg::String>(
      "my_topic", 10,
      std::bind(&SubscriberNode::subscriberCallback, this,
                std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Subscriber Created");
}

void SubscriberNode::subscriberCallback(
    example_interfaces::msg::String::SharedPtr msg) {
  // Log the received message
  RCLCPP_INFO(this->get_logger(), "%s", msg->data.c_str());

  // Special handling for updated messages
  if (msg->data == "This msg has been updated!") {
    RCLCPP_WARN(this->get_logger(), "Updated message received.");
  }
}

/**
 * @brief Main function to initialize and run the ROS2 subscriber node
 * @param argc Number of command line arguments
 * @param argv Array of command line arguments
 * @return Exit status
 */
int main(int argc, char **argv) {
  // Initialize ROS2 communication
  rclcpp::init(argc, argv);

  // Create and initialize the subscriber node
  auto node = std::make_shared<SubscriberNode>();

  RCLCPP_DEBUG(node->get_logger(), "Subscriber Node has started spinning");

  // Keep the node running
  rclcpp::spin(node);

  // Cleanup and shutdown
  rclcpp::shutdown();
  return 0;
}
