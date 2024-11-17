# ROS 2 Tutorial

This project implements a ROS 2 system with a publisher node (`MyNode`) and a subscriber node (`SubscriberNode`). The publisher sends string messages to the topic `my_topic` and provides a service to update the message content. The subscriber receives and processes these messages.

---

## Assumptions and Dependencies
- **ROS 2 Humble**: This project was developed and tested with ROS 2 Humble. Ensure it's installed and sourced.
- **C++17 or later**: Required for standard library components used in this code.
- **Dependencies**:
  - `rclcpp`: ROS 2 C++ client library for node management, logging, and communication.
  - `example_interfaces`: Provides the `String` message type and `SetBool` service type.
  - `tf2_ros`: Required for handling transforms.
  - `rosbag2`: For recording and replaying bag files.

---

## Features
### Publisher Node (`MyNode`)
- Publishes string messages to `my_topic`.
- Configurable publishing rate via ROS parameters.
- Provides a service (`update_publisher`) to modify the published message.
- Includes comprehensive logging at various levels.

### Subscriber Node (`SubscriberNode`)
- Subscribes to messages on `my_topic`.
- Processes received messages with special handling for updated content.
- Implements informative logging.

### Transform Broadcasting
- Publishes a transform with the parent frame `world` and child frame `talk`.

---

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

### 9. Working with Bag Files
#### Record Bag Files
To record a bag file for 15 seconds:

   ```sh
   ros2 launch beginner_tutorials ros_bags.launch.py record_bag:=True
   ```
The recorded bag files will be stored in the results/bag_files directory of beginner_tutorials package.

#### Inspect Bag Files
To inspect the recorded bag file:

   ```sh
   ros2 bag info <path_to_package>/src/results/bag_files
   ```

#### Playback Bag Files
To play back a recorded bag file:

   ```sh
   ros2 bag play <path_to_package>/src/results/bag_files/<bag_file_name>
   ```

#### Verify Playback with Listener Node
Run the subscriber node during playback to verify the recorded messages:
   ```sh
   ros2 run beginner_tutorials subscriber_node
   ```

### 10. Working with Bag Files
#### View Transform Frames
To list the TF frames in the system:
   ```sh
   ros2 run tf2_tools view_frames
   ```
#### Echo Transform
To observe the transform from world to talk:
   ```sh
   ros2 run tf2_ros tf2_echo world talk
   ```
### 11. ROS Testing
#### Run ROS Tests
The package includes unit tests. Run them using:
   ```sh
   colcon test --packages-select beginner_tutorials
   ```

#### Alternate way of running test
Run the following in root of workspace
   ```sh
   ./build/beginner_tutorials/my_node_test
   ```


## clang-tidy output
![clang-tidy](https://github.com/user-attachments/assets/4df1e261-4c23-47bb-9be6-8a15c8e70992)



## clanglint output
![cpplint](https://github.com/user-attachments/assets/1a66fe5d-c45e-4298-8f5d-6ad4509ba13e)


## rqt_console

### INFO
![info](https://github.com/user-attachments/assets/ef75042a-1403-4597-90b7-f32b070090cd)


### WARN
![warning](https://github.com/user-attachments/assets/1b6e5c41-e38c-427f-9138-dcd91da45e74)


