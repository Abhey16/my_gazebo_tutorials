# ROS 2 Publisher Node - MyNode

This project implements a simple ROS 2 node, `MyNode`, that publishes a string message to the topic `my_topic` every second. 
The message published is "Hi, My name is Abhey." This project uses ROS 2 Humble and includes necessary dependencies for building and running the node.

## Assumptions and Dependencies

- **ROS 2 Humble**: Make sure ROS 2 Humble is installed and sourced. This project was developed and tested with ROS 2 Humble.
- **C++17 or later**: Required for standard library components used in this code.
- **Dependencies**:
  - `rclcpp`: The ROS 2 C++ client library, used for node management, logging, and publishing.
  - `example_interfaces`: Provides the `String` message type used in this project.

## Code Overview

The `MyNode` class:

- Initializes a publisher to the `my_topic` topic.
- Sets up a timer to publish a message every second.
- Logs the published message to the console.

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

### 6. Run the Publisher Node:
   ```sh
   ros2 run beginner_tutorials my_node
   ```

## clang-tidy output
![Screenshot from 2024-11-07 22-55-59](https://github.com/user-attachments/assets/939077c3-5959-4ba4-b9bb-4a448ce4bae3)


## clanglint output
![Screenshot from 2024-11-07 23-02-07](https://github.com/user-attachments/assets/9ac670d0-c5e1-466e-8aeb-b59ce4812a83)
