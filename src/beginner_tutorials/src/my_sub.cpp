#include "beginner_tutorials/my_sub.hpp"
#include <example_interfaces/msg/detail/string__struct.hpp>
#include <memory>
#include <rclcpp/logging.hpp>

SubscriberNode::SubscriberNode() : Node("my_sub")
{
    subscriber_ = this->create_subscription<example_interfaces::msg::String>("my_topic", 10,
                                                                            std::bind(&SubscriberNode::subscriberCallback,
                                                                            this,std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(),"Subscriber Created");
}

void SubscriberNode::subscriberCallback(example_interfaces::msg::String::SharedPtr msg)
{

    RCLCPP_INFO(this->get_logger(), "%s", msg->data.c_str());

    if (msg->data == "This msg has been updated!") 
    {
        RCLCPP_WARN(this->get_logger(), "Updated message received.");
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);

    auto node = std::make_shared<SubscriberNode>();

    RCLCPP_DEBUG(node->get_logger(), "Subscriber Node has started spinning");

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}