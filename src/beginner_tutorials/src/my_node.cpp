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

// Constructor of MyNode class
MyNode::MyNode() : Node("my_node"),base_message_{"Hi, My name is Abhey"}
{
  // Declare a parameter for publisher time_interval in seconds
  this->declare_parameter("publish_time", 1);
  int publish_time;
  this->get_parameter("publish_time", publish_time);

  // Creating publisher
  publisher_ =this->create_publisher<example_interfaces::msg::String>("my_topic", 10);

  // Creating timer which calls publishCallback function every 1s
  timer_ = this->create_wall_timer(std::chrono::seconds(publish_time),
                                   std::bind(&MyNode::publishCallback, this));

  // Creating server
  server_ = this->create_service<example_interfaces::srv::SetBool>("update_publisher", 
                                                                    std::bind(&MyNode::serverCallback,this,_1,_2));

  RCLCPP_INFO(this->get_logger(), "Server and Publisher Created");
}

void MyNode::publishCallback() {
  // Creating string object
  auto msg = example_interfaces::msg::String();
  msg.data = base_message_;

  // Printing the msg that is published on "my_topic"
  // RCLCPP_DEBUG_STREAM(this->get_logger(), "Publishing: " << msg.data);
  RCLCPP_INFO(this->get_logger(), "%s", msg.data.c_str());

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

int main(int argc, char** argv) {
  // Initializing ros communication
  rclcpp::init(argc, argv);

  // Node created
  auto node = std::make_shared<MyNode>();

  // Keeping the node alive
  rclcpp::spin(node);

  // killing the node
  rclcpp::shutdown();

  return 0;
}
