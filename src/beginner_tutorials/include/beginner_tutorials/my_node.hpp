/**
 * @file my_node.hpp
 * @brief Header file for the MyNode class implementing a ROS2 node with
 * publisher and service
 * @details This class creates a ROS2 node that publishes custom string messages
 * and provides a service to update the published message
 */

#pragma once

#include <tf2_ros/static_transform_broadcaster.h>

#include <string>
#include <memory>

#include <example_interfaces/msg/detail/string__struct.hpp>
#include <example_interfaces/msg/string.hpp>
#include <example_interfaces/srv/detail/set_bool__struct.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/timer.hpp>


/*
 * @class MyNode
 * @brief A ROS2 node class that implements a string publisher and boolean
 * service
 * @details This node periodically publishes a custom string message and
 * provides a service to toggle between different message contents
 */
class MyNode : public rclcpp::Node {
 public:
  /**
   * @brief Constructor for the MyNode class
   * @details Initializes the publisher, timer, and service server with default
   * configurations
   */
  explicit MyNode(char* transformation[]);

 private:
  /**
   * @brief Callback function for the timer to publish messages
   * @details Publishes the current base_message_ string to the "my_topic" topic
   */
  void publishCallback();

  /**
   * @brief Callback function for the service server
   * @param request SharedPtr to the service request containing a boolean flag
   * @param response SharedPtr to the service response indicating success status
   * @details Updates the base_message_ based on the request's boolean value
   */
  void serverCallback(
      const example_interfaces::srv::SetBool::Request::SharedPtr request,
      const example_interfaces::srv::SetBool::Response::SharedPtr response);

  rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr
      publisher_;                       ///< Publisher for string messages
  rclcpp::TimerBase::SharedPtr timer_;  ///< Timer for periodic publishing
  rclcpp::Service<example_interfaces::srv::SetBool>::SharedPtr
      server_;  ///< Service server for message updates
  std::string
      base_message_;  ///< Storage for the current message to be published
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster>
      tf_static_broadcaster_;  // broadcaster

  void make_transforms(char* transformation[]);
};
