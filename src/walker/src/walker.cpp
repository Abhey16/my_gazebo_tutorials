// MIT License
//
// Copyright (c) 2024 Abhey Sharma

/**
 * @file walker.cpp
 * @brief Implementation of the RobotController and its states for obstacle
 * avoidance.
 */

#include "walker/walker.hpp"

/**
 * @brief Constructor for the RobotController class.
 *
 * Initializes the ROS 2 node, sets up subscriptions for laser scan data,
 * publishers for velocity commands, and the initial robot state.
 */
RobotController::RobotController()
    : Node("robot_controller"), turn_clockwise_(true) {
  laser_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      std::bind(&RobotController::laserScanCallback, this,
                std::placeholders::_1));

  velocity_publisher_ =
      this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  active_state_ = std::make_shared<ForwardState>();
}

/**
 * @brief Sets the current state of the robot.
 * @param state Shared pointer to the new robot state.
 */
void RobotController::setState(std::shared_ptr<RobotState> state) {
  active_state_ = state;
}

/**
 * @brief Executes the logic of the current robot state.
 */
void RobotController::process() {
  if (active_state_) {
    active_state_->execute(*this);
  }
}

/**
 * @brief Publishes velocity commands to drive the robot.
 *
 * @param linear_speed Linear velocity in meters per second.
 * @param angular_speed Angular velocity in radians per second.
 */
void RobotController::drive(double linear_speed, double angular_speed) {
  geometry_msgs::msg::Twist twist_message;
  twist_message.linear.x = linear_speed;
  twist_message.angular.z = angular_speed;
  velocity_publisher_->publish(twist_message);
}

/**
 * @brief Checks if an obstacle is detected nearby.
 * @return True if an obstacle is detected; false otherwise.
 */
bool RobotController::isObstacleNearby() const { return obstacle_present_; }

/**
 * @brief Toggles the robot's turn direction.
 * @return True if the robot will now turn clockwise; false if counterclockwise.
 */
bool RobotController::switchTurnDirection() {
  turn_clockwise_ = !turn_clockwise_;
  return turn_clockwise_;
}

/**
 * @brief Callback for processing laser scan data.
 *
 * Determines if an obstacle is nearby by analyzing the laser scan ranges.
 * If an obstacle is detected, updates the obstacle flag and triggers state
 * execution.
 *
 * @param msg Shared pointer to the LaserScan message.
 */
void RobotController::laserScanCallback(
    const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  obstacle_present_ = false;

  // Check a subset of laser scan readings for nearby obstacles
  for (size_t i = 0; i < 30; ++i) {
    if (msg->ranges[i] < 0.8 || msg->ranges[msg->ranges.size() - 1 - i] < 0.8) {
      obstacle_present_ = true;
      break;
    }
  }

  if (obstacle_present_) {
    RCLCPP_INFO(this->get_logger(), "Obstacle detected!");
  }

  process();
}

/**
 * @brief Executes the logic for the ForwardState.
 *
 * The robot moves forward unless an obstacle is detected, in which case
 * it transitions to the TurnState.
 *
 * @param controller Reference to the RobotController managing the robot.
 */
void ForwardState::execute(RobotController& controller) {
  if (controller.isObstacleNearby()) {
    controller.setState(
        std::make_shared<TurnState>(controller.switchTurnDirection()));
  } else {
    controller.drive(0.2, 0.0);
  }
}

/**
 * @brief Executes the logic for the TurnState.
 *
 * The robot turns in a specified direction until the obstacle is no longer
 * detected, then transitions back to the ForwardState.
 *
 * @param controller Reference to the RobotController managing the robot.
 */
void TurnState::execute(RobotController& controller) {
  if (!controller.isObstacleNearby()) {
    controller.setState(std::make_shared<ForwardState>());
  } else {
    RCLCPP_INFO(controller.get_logger(), turn_clockwise_
                                             ? "Turning clockwise"
                                             : "Turning counterclockwise");
    controller.drive(0.0, turn_clockwise_ ? -0.5 : 0.5);
  }
}

/**
 * @brief Main function to initialize and run the RobotController node.
 *
 * @param argc Number of command-line arguments.
 * @param argv Array of command-line arguments.
 * @return Exit status of the program.
 */
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto controller = std::make_shared<RobotController>();
  rclcpp::spin(controller);
  rclcpp::shutdown();
  return 0;
}
