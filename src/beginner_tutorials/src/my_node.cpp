#include "beginner_tutorials/my_node.hpp"

#include <chrono>
#include <memory>

#include <example_interfaces/msg/detail/string__struct.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/utilities.hpp>



// Constructor of MyNode class
MyNode::MyNode() : Node("my_node") {
  // Creating publisher
  publisher_ =
      this->create_publisher<example_interfaces::msg::String>("my_topic", 10);

  // Creating timer which calls publishCallback function every 1s
  timer_ = this->create_wall_timer(std::chrono::seconds(1),
                                   std::bind(&MyNode::publishCallback, this));

  RCLCPP_INFO(this->get_logger(), "Publisher Created");
}

void MyNode::publishCallback() {
  // Creating string object
  auto msg = example_interfaces::msg::String();
  msg.data = "Hi, My name is Abhey";

  // Printing the msg that is published on "my_topic"
  RCLCPP_INFO(this->get_logger(), "%s", msg.data.c_str());

  // publish
  publisher_->publish(msg);
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
