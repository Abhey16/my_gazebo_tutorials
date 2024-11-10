/**
 * @file my_sub.hpp
 * @brief Header file for the SubscriberNode class implementing a ROS2
 * subscriber node
 * @details This class creates a ROS2 node that subscribes to custom string
 * messages published on the "my_topic" topic
 */

#pragma once

#include <example_interfaces/msg/detail/string__struct.hpp>
#include <example_interfaces/msg/string.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>

/**
 * @class SubscriberNode
 * @brief A ROS2 node class that implements a string message subscriber
 * @details This node subscribes to string messages and processes them,
 *          providing special handling for updated messages
 */
class SubscriberNode : public rclcpp::Node {
 public:
  /**
   * @brief Constructor for the SubscriberNode class
   * @details Initializes the subscriber with default configurations
   */
  SubscriberNode();

 private:
  /**
   * @brief Callback function for processing received messages
   * @param msg SharedPtr to the received string message
   * @details Processes incoming messages and logs them, with special handling
   *          for messages indicating an update
   */
  void subscriberCallback(example_interfaces::msg::String::SharedPtr msg);

  rclcpp::Subscription<example_interfaces::msg::String>::SharedPtr
      subscriber_;  ///< Subscriber for string messages
};
