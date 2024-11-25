#include "walker/walker.hpp"

// Constructor for RobotController
RobotController::RobotController() : Node("robot_controller"), turn_clockwise_(true) {
  laser_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&RobotController::laserScanCallback, this, std::placeholders::_1));

  velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  active_state_ = std::make_shared<ForwardState>();
}

// Set the current state of the robot
void RobotController::setState(std::shared_ptr<RobotState> state) {
  active_state_ = state;
}

// Perform state execution
void RobotController::process() {
  if (active_state_) {
    active_state_->execute(*this);
  }
}

// Publish velocity commands to drive the robot
void RobotController::drive(double linear_speed, double angular_speed) {
  geometry_msgs::msg::Twist twist_message;
  twist_message.linear.x = linear_speed;
  twist_message.angular.z = angular_speed;
  velocity_publisher_->publish(twist_message);
}

// Check if an obstacle is detected
bool RobotController::isObstacleNearby() const {
  return obstacle_present_;
}

// Toggle the turn direction (clockwise or counterclockwise)
bool RobotController::switchTurnDirection() {
  turn_clockwise_ = !turn_clockwise_;
  return turn_clockwise_;
}

// Callback for processing laser scan data
void RobotController::laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
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

// Handle logic in the ForwardState
void ForwardState::execute(RobotController& controller) {
  if (controller.isObstacleNearby()) {
    controller.setState(std::make_shared<TurnState>(controller.switchTurnDirection()));
  } else {
    controller.drive(0.2, 0.0);
  }
}

// Handle logic in the TurnState
void TurnState::execute(RobotController& controller) {
  if (!controller.isObstacleNearby()) {
    controller.setState(std::make_shared<ForwardState>());
  } else {
    RCLCPP_INFO(controller.get_logger(), turn_clockwise_ ? "Turning clockwise" : "Turning counterclockwise");
    controller.drive(0.0, turn_clockwise_ ? -0.5 : 0.5);
  }
}

// Main function to start the node
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto controller = std::make_shared<RobotController>();
  rclcpp::spin(controller);
  rclcpp::shutdown();
  return 0;
}
