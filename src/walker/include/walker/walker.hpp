// MIT License
//
// Copyright (c) 2024 Abhey Sharma

/**
 * @file walker.hpp
 * @brief Contains the definitions of the RobotController and RobotState classes
 *        and their derived classes for implementing a state design pattern in a
 * robot.
 */

#pragma once
#include <memory>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

// Forward declaration of RobotController
class RobotController;

/**
 * @class RobotState
 * @brief Abstract base class representing a state of the robot.
 */
class RobotState {
 public:
  /**
   * @brief Virtual destructor for the RobotState class.
   */
  virtual ~RobotState() = default;

  /**
   * @brief Pure virtual function to handle state-specific logic.
   * @param controller Reference to the RobotController managing the robot.
   */
  // Intentionally modifying the RobotController.
  virtual void execute(RobotController& controller) = 0;  // NOLINT
  // (runtime/references)
};

/**
 * @class ForwardState
 * @brief Represents the robot state when it is moving forward.
 */
class ForwardState : public RobotState {
 public:
  /**
   * @brief Executes the behavior for the forward-moving state.
   * @param controller Reference to the RobotController managing the robot.
   */
  void execute(RobotController& controller) override;
};

/**
 * @class TurnState
 * @brief Represents the robot state when it is turning to avoid obstacles.
 */
class TurnState : public RobotState {
 public:
  /**
   * @brief Constructor for TurnState.
   * @param clockwise If true, the robot turns clockwise; otherwise,
   * counterclockwise.
   */
  explicit TurnState(bool clockwise) : turn_clockwise_(clockwise) {}

  /**
   * @brief Executes the behavior for the turning state.
   * @param controller Reference to the RobotController managing the robot.
   */
  void execute(RobotController& controller) override;

 private:
  bool turn_clockwise_;  ///< Indicates the direction of the turn.
};

/**
 * @class RobotController
 * @brief Main controller class for the robot, managing its states and sensors.
 */
class RobotController : public rclcpp::Node {
 public:
  /**
   * @brief Constructs a RobotController node.
   */
  RobotController();

  /**
   * @brief Sets the active state of the robot.
   * @param state A shared pointer to the new state.
   */
  void setState(std::shared_ptr<RobotState> state);

  /**
   * @brief Main process function that executes the current state.
   */
  void process();

  /**
   * @brief Sends drive commands to the robot.
   * @param linear_speed The linear speed for the robot.
   * @param angular_speed The angular speed for the robot.
   */
  void drive(double linear_speed, double angular_speed);

  /**
   * @brief Checks if there is an obstacle nearby.
   * @return True if an obstacle is detected; false otherwise.
   */
  bool isObstacleNearby() const;

  /**
   * @brief Toggles the robot's turning direction.
   * @return True if turning clockwise; false if counterclockwise.
   */
  bool switchTurnDirection();

 private:
  /**
   * @brief Callback function for processing laser scan data.
   * @param msg Shared pointer to the received LaserScan message.
   */
  void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      laser_subscription_;  ///< Subscription to laser scan data.
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr
      velocity_publisher_;  ///< Publisher for velocity commands.

  std::shared_ptr<RobotState>
      active_state_;  ///< Pointer to the current active state.
  bool obstacle_present_{
      false};                  ///< Flag indicating if an obstacle is detected.
  bool turn_clockwise_{true};  ///< Current turning direction.
};
