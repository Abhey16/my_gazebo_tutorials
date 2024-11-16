// MIT License
//
// Copyright (c) 2024 Abhey Sharma

#include <catch_ros2/catch_ros2.hpp>
#include <chrono>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;
using std_msgs::msg::String;


////////////////////////////////////////////////
// Fixture Definition
////////////////////////////////////////////////
auto TestLogger = rclcpp::get_logger("TestLogger"); // Define a global logger

class IntegrationTestSetup {
public:
  IntegrationTestSetup() {

    // Initialize a node for integration testing:

    integrationTestNode = rclcpp::Node::make_shared("IntegrationTestNode");
    TestLogger = integrationTestNode->get_logger(); // Ensure logs appear correctly


    // Declare a parameter to specify the test duration:

    integrationTestNode->declare_parameter<double>("test_run_time", 10.0);


    // Retrieve the parameter value:
 
    testRunDuration = integrationTestNode->get_parameter("test_run_time").get_parameter_value().get<double>();
    RCLCPP_INFO_STREAM(TestLogger, "Test duration set to " << testRunDuration << " seconds.");
  }

  ~IntegrationTestSetup() {}

protected:
  double testRunDuration;
  rclcpp::Node::SharedPtr integrationTestNode;
};

////////////////////////////////////////////////
// Test Case: PublisherNode Behavior
////////////////////////////////////////////////
TEST_CASE_METHOD(IntegrationTestSetup, "PublisherNode message broadcasting", "[publisher]") {
  // Create a listener node for the "chatter" topic
  auto listenerNode = rclcpp::Node::make_shared("listener_node");
  bool isMessageReceived = false;

  auto subscriber = listenerNode->create_subscription<std_msgs::msg::String>(
      "chatter", 10,
      [&isMessageReceived](std_msgs::msg::String::ConstSharedPtr msg) {
        // Validate the message content
        REQUIRE(msg->data == "Hi, My name is Abhey");
        isMessageReceived = true;
      });

  // Create a broadcaster node to send messages
  auto broadcasterNode = rclcpp::Node::make_shared("broadcaster_node");
  auto messagePublisher = broadcasterNode->create_publisher<std_msgs::msg::String>("chatter", 10);

  // Initialize an executor to run both nodes
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(listenerNode);
  executor.add_node(broadcasterNode);

  // Allow some time for the nodes to initialize
  std::this_thread::sleep_for(std::chrono::seconds(1));

  // Simulate the behavior of the Talker node by publishing a message
  auto testMessage = std_msgs::msg::String();
  testMessage.data = "Hi, My name is Abhey";
  messagePublisher->publish(testMessage);

  // Spin nodes for a set duration to verify message exchange
  auto testStartTime = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - testStartTime < std::chrono::seconds(static_cast<int>(testRunDuration))) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if (isMessageReceived) {
      break;
    }
  }

  // Confirm that the listener node received the message
  CHECK(isMessageReceived);
}
