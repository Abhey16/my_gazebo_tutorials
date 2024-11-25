# pragma once

#include <memory>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

// Forward declaration of RobotController
class RobotController;

// Abstract state class for the robot
class RobotState {
public:
  virtual ~RobotState() = default;

  // Handle logic for transitioning between states
  virtual void execute(RobotController& controller) = 0;
};

// State when the robot is moving forward
class ForwardState : public RobotState {
public:
  void execute(RobotController& controller) override;
};

// State when the robot is turning to avoid obstacles
class TurnState : public RobotState {
public:
  explicit TurnState(bool clockwise) : turn_clockwise_(clockwise) {}

  void execute(RobotController& controller) override;

private:
  bool turn_clockwise_;
};

// Main controller for the robot
class RobotController : public rclcpp::Node {
public:
  RobotController();

  void setState(std::shared_ptr<RobotState> state);

  void process();

  void drive(double linear_speed, double angular_speed);

  bool isObstacleNearby() const;

  bool switchTurnDirection();

private:
  void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;

  std::shared_ptr<RobotState> active_state_;
  bool obstacle_present_{false};
  bool turn_clockwise_{true};
};

