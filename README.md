# ROS 2 Publisher/Subscriber Tutorial
This project implements a ROS 2 system with a publisher node (`MyNode`) and a subscriber node (`SubscriberNode`). The publisher sends string messages to the topic `my_topic` and provides a service to update the message content. The subscriber receives and processes these messages.

## Assumptions and Dependencies
- **ROS 2 Humble**: This project was developed and tested with ROS 2 Humble. Make sure it's installed and sourced.
- **C++17 or later**: Required for standard library components used in this code.
- **Dependencies**:
  - `rclcpp`: The ROS 2 C++ client library for node management, logging, and communication.
  - `example_interfaces`: Provides the `String` message type and `SetBool` service type.

## Features
### Publisher Node (`MyNode`)
- Publishes string messages to `my_topic`
- Configurable publishing rate via ROS parameter
- Provides a service (`update_publisher`) to modify the published message
- Includes comprehensive logging at various levels

### Subscriber Node (`SubscriberNode`)
- Subscribes to messages on `my_topic`
- Processes received messages with special handling for updated content
- Implements informative logging

## Build and Run Instructions

### 1. Clone the Repository
   ```sh
   git clone https://github.com/Abhey16/my_beginner_tutorials.git
   ```
### 2. Navigate to Your ROS 2 Workspace
   ```sh
   cd ~/my_beginner_tutorials
   ```
### 3. Build the Package:
   ```sh
   colcon build --packages-select beginner_tutorials
   ```
### 4. Source the Workspace:
   After building, source the workspace to add the package to your environment:
   ```sh
   source install/setup.bash
  ```
### 5. Navigate to Your Workspace:
   ```sh
   cd ~/my_beginner_tutorials
   ```

### 6. Run the Nodes:
#### Run the Publisher Node
   ```sh
   ros2 run beginner_tutorials my_node
   ```
#### Run the Subscriber Node (in a new terminal)
   ```sh
   ros2 run beginner_tutorials my_node
   ```
### 7. Using the Service:
The publisher node provides a service to update the published message. You can call it using the command-line tool:

To update the message:
   ```sh
   ros2 service call /update_publisher example_interfaces/srv/SetBool "data: true"
   ```

To revert to the original message:
   ```sh
   ros2 service call /update_publisher example_interfaces/srv/SetBool "data: true"
   ```
### 8. Launch File Usage
The package includes a launch file to start both nodes simultaneously. To use it:

   ```sh
   ros2 launch beginner_tutorials tutorial_launch.py
   ```
Launch file parameters:
* Modify publishing rate:

```sh
ros2 launch beginner_tutorials tutorial_launch.py publish_time:=2
```

## clang-tidy output
![clang-tidy](https://github.com/user-attachments/assets/21864b94-3c91-45cf-bb19-c11420b19348)


## clanglint output
![cpplint](https://github.com/user-attachments/assets/aa165bc3-033a-4acd-95c8-9e7eaefe9971)


## rqt_console

### INFO
![info](https://github.com/user-attachments/assets/ef75042a-1403-4597-90b7-f32b070090cd)


### WARN
![warning](https://github.com/user-attachments/assets/1b6e5c41-e38c-427f-9138-dcd91da45e74)


