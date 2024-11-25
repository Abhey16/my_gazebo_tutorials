// MIT License
//
// Copyright (c) 2024 Abhey Sharma

/**
 * @file my_node.cpp
 * @brief Implementation file for the MyNode class.
 * @details Contains the implementation of the MyNode class methods including
 *          the constructor, callbacks, and the main function to run the ROS 2
 * node.
 */

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>

// C system headers
#include <cstdlib>

// C++ system headers
#include <chrono>  // NOLINT(build/c++11)
#include <functional>
#include <memory>

#include "beginner_tutorials/my_node.hpp"

// Other library headers
#include <example_interfaces/msg/detail/string__struct.hpp>
#include <example_interfaces/srv/detail/set_bool__struct.hpp>
#include <example_interfaces/srv/set_bool.hpp>
#include <geometry_msgs/msg/detail/transform_stamped__struct.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/utilities.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

/**
 * @brief Constructor for MyNode.
 * @param transformation Array of command-line arguments for transformation
 * parameters.
 */
MyNode::MyNode(char* transformation[])
    : Node("my_node"), base_message_{"Hi, My name is Abhey"} {
  // Declare a parameter for publisher time_interval in seconds
  this->declare_parameter("publish_time", 1);
  int publish_time;
  this->get_parameter("publish_time", publish_time);

  // Creating publisher
  publisher_ =
      this->create_publisher<example_interfaces::msg::String>("my_topic", 10);

  // Creating timer which calls publishCallback function every publish_time
  // seconds
  timer_ = this->create_wall_timer(std::chrono::seconds(publish_time),
                                   std::bind(&MyNode::publishCallback, this));

  // Creating server
  server_ = this->create_service<example_interfaces::srv::SetBool>(
      "update_publisher", std::bind(&MyNode::serverCallback, this, _1, _2));

  // Creating broadcaster
  tf_static_broadcaster_ =
      std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

  // Broadcasting
  this->make_transforms(transformation);

  RCLCPP_INFO(this->get_logger(), "Server, Publisher and Broadcaster Created");
}

/**
 * @brief Publishes a message to the topic.
 * @details Uses the current base message and publishes it to the "my_topic"
 * topic.
 */
void MyNode::publishCallback() {
  if (!publisher_) {
    // Critical failure case: publisher is null
    RCLCPP_FATAL(this->get_logger(),
                 "Publisher not initialized! Node cannot continue.");
    rclcpp::shutdown();  // Shutdown node on fatal error
    return;
  }

  // Creating string object
  auto msg = example_interfaces::msg::String();
  msg.data = base_message_;

  // Printing the msg that is published on "my_topic"
  RCLCPP_DEBUG_STREAM(this->get_logger(), "Publishing: " << msg.data);

  // Publish
  publisher_->publish(msg);
}

/**
 * @brief Callback function for the update_publisher service.
 * @param request Shared pointer to the service request.
 * @param response Shared pointer to the service response.
 */
void MyNode::serverCallback(
    const example_interfaces::srv::SetBool::Request::SharedPtr request,
    const example_interfaces::srv::SetBool::Response::SharedPtr response) {
  if (request->data) {
    base_message_ = "This msg has been updated!";
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Base message changed successfully.");
    response->success = true;
  } else {
    RCLCPP_WARN(this->get_logger(), "Received request to revert message.");
    base_message_ = "Hi, My name is Abhey";
    response->success = false;
  }

  RCLCPP_DEBUG(this->get_logger(), "Service request processed.");
}

/**
 * @brief Creates and broadcasts a static transformation.
 * @param transformation Array containing transformation parameters.
 */
void MyNode::make_transforms(char* transformation[]) {
  geometry_msgs::msg::TransformStamped t;

  t.header.stamp = this->get_clock()->now();
  t.header.frame_id = "world";
  t.child_frame_id = transformation[1];

  t.transform.translation.x = atof(transformation[2]);
  t.transform.translation.y = atof(transformation[3]);
  t.transform.translation.z = atof(transformation[4]);

  tf2::Quaternion q;

  q.setRPY(atof(transformation[5]), atof(transformation[6]),
           atof(transformation[7]));

  t.transform.rotation.w = q.w();
  t.transform.rotation.x = q.x();
  t.transform.rotation.y = q.y();
  t.transform.rotation.z = q.z();

  tf_static_broadcaster_->sendTransform(t);
}

/**
 * @brief Main function to initialize and run the ROS 2 node.
 * @param argc Number of command-line arguments.
 * @param argv Array of command-line arguments.
 * @return Exit status.
 */
int main(int argc, char** argv) {
  auto logger = rclcpp::get_logger("logger");

  // Obtain parameters from command line arguments
  if (argc < 8) {
    RCLCPP_INFO(logger,
                "Invalid number of parameters\nusage: "
                "$ ros2 run learning_tf2_cpp static_turtle_tf2_broadcaster "
                "child_frame_name x y z roll pitch yaw");
    return 1;
  }

  // As the parent frame of the transform is `world`, it is
  // necessary to check that the frame name passed is different
  if (strcmp(argv[1], "world") == 0) {
    RCLCPP_INFO(logger, "Your static turtle name cannot be 'world'");
    return 2;
  }

  // Initializing ROS communication
  rclcpp::init(argc, argv);

  // Node created
  auto node = std::make_shared<MyNode>(argv);

  RCLCPP_DEBUG(node->get_logger(), "Publisher Node has started spinning");

  // Keeping the node alive
  rclcpp::spin(node);

  // Killing the node
  rclcpp::shutdown();

  return 0;
}
