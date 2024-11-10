/**
 * @file my_node.cpp
 * @brief Implementation file for the MyNode class
 * @details Contains the implementation of the MyNode class methods including
 *          constructor, callbacks, and main function
 */

#include "beginner_tutorials/my_node.hpp"

#include <chrono>
#include <example_interfaces/srv/detail/set_bool__struct.hpp>
#include <functional>
#include <memory>

#include <example_interfaces/msg/detail/string__struct.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/utilities.hpp>
#include <example_interfaces/srv/set_bool.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

MyNode::MyNode() : Node("my_node"), base_message_{"Hi, My name is Abhey"}
{
  // Declare a parameter for publisher time_interval in seconds
  this->declare_parameter("publish_time", 1);
  int publish_time;
  this->get_parameter("publish_time", publish_time);

  // Creating publisher
  publisher_ = this->create_publisher<example_interfaces::msg::String>("my_topic", 10);

  // Creating timer which calls publishCallback function every publish_time seconds
  timer_ = this->create_wall_timer(std::chrono::seconds(publish_time),
                                   std::bind(&MyNode::publishCallback, this));

  // Creating server
  server_ = this->create_service<example_interfaces::srv::SetBool>("update_publisher", 
                                                                    std::bind(&MyNode::serverCallback,this,_1,_2));

  RCLCPP_INFO(this->get_logger(), "Server and Publisher Created");
}

void MyNode::publishCallback() {
  if (!publisher_) {
    // Critical failure case: publisher is null
    RCLCPP_FATAL(this->get_logger(), "Publisher not initialized! Node cannot continue.");
    rclcpp::shutdown();  // Shutdown node on fatal error
    return;
  }

  // Creating string object
  auto msg = example_interfaces::msg::String();
  msg.data = base_message_;

  // Printing the msg that is published on "my_topic"
  RCLCPP_DEBUG_STREAM(this->get_logger(), "Publishing: " << msg.data);
  
  // publish
  publisher_->publish(msg);
}

void MyNode::serverCallback(const example_interfaces::srv::SetBool::Request::SharedPtr request, 
                            const example_interfaces::srv::SetBool::Response::SharedPtr response)
{
  if(request->data)
  {
    base_message_ = "This msg has been updated!";
    RCLCPP_INFO_STREAM(this->get_logger(), "Base message changed successfully.");
    response->success = true;
  }
  else
  {
    RCLCPP_WARN(this->get_logger(), "Received request to revert message.");
    base_message_ = "Hi, My name is Abhey";
    response->success = false;
  }

  RCLCPP_DEBUG(this->get_logger(), "Service request processed.");
}

/**
 * @brief Main function to initialize and run the ROS2 node
 * @param argc Number of command line arguments
 * @param argv Array of command line arguments
 * @return Exit status
 */
int main(int argc, char** argv) {
  // Initializing ros communication
  rclcpp::init(argc, argv);

  // Node created
  auto node = std::make_shared<MyNode>();

  RCLCPP_DEBUG(node->get_logger(), "Publisher Node has started spinning");

  // Keeping the node alive
  rclcpp::spin(node);

  // killing the node
  rclcpp::shutdown();

  return 0;
}