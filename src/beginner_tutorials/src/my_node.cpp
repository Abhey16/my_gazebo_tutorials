#include "beginner_tutorials/my_node.hpp"
#include <chrono>
#include <example_interfaces/msg/detail/string__struct.hpp>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/utilities.hpp>


MyNode::MyNode(): Node("my_node")
{
    publisher_ = this->create_publisher<example_interfaces::msg::String>("my_topic", 10);

    timer_ = this->create_wall_timer(std::chrono::seconds(1),
                                        std::bind(&MyNode::publishCallback,this));

    RCLCPP_INFO(this->get_logger(),"Publisher Created");
}

void MyNode::publishCallback()
{
    auto msg = example_interfaces::msg::String();
    msg.data = "Hi, My name is Abhey";
    RCLCPP_INFO(this->get_logger(),"%s",msg.data.c_str());
    publisher_->publish(msg);
}


int main(int argc, char** argv)
{
    rclcpp::init(argc,argv);

    auto node = std::make_shared<MyNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}