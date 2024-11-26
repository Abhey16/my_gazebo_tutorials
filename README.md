# ROS 2 Walker Tutorial

This project implements a ROS 2 system with a walker algorithm for a robot in Gazebo. The walker package is designed for controlling a robot's movement and simulating its behavior in the Gazebo environment.

---

## Assumptions and Dependencies
- **ROS 2 Humble**: This project was developed and tested with ROS 2 Humble. Ensure it's installed and sourced.
- **C++17 or later**: Required for standard library components used in this code.
- **Dependencies**:
  - `rclcpp`: ROS 2 C++ client library for node management, logging, and communication.
  - `tf2_ros`: Required for handling transforms.
  - `rosbag2`: For recording and replaying bag files.
  - `walker`: Provides robot movement control algorithms.

---

## Features
### Walker Node
- Implements the walker algorithm for controlling a robot's movement in Gazebo.
- Uses sensor data and pre-defined parameters to navigate and interact with the environment.
- Provides logging for status and errors.

### Gazebo Simulation
- Simulates the robot's movements using the walker algorithm in a custom Gazebo world.
- The walker algorithm ensures that the robot can traverse various terrains and obstacles.

---

## Build and Run Instructions

### 1. Clone the Repository
   ```sh
   git clone https://github.com/Abhey16/my_gazebo_tutorials.git
   ```
### 2. Navigate to Your ROS 2 Workspace
   ```sh
   cd ~/my_gazebo_tutorials
   ```
### 3. Build the Package:
   ```sh
   colcon build --packages-select walker
   ```
### 4. Source the Workspace:
   After building, source the workspace to add the package to your environment:
   ```sh
   source install/setup.bash
  ```
### 5. Navigate to Your Workspace:
   ```sh
   cd ~/my_gazebo_tutorials
   ```

### 6. Run the Nodes:
#### Run the Walker Node
   ```sh
   ros2 run walker walker_node

   ```

### 7. Launch File Usage
#### walker
```sh
ros2 launch walker world.launch.py
```

### 8. Working with Bag Files
#### Record Bag Files

#### beginners_tutorial
To record a bag file for 15 seconds:

#### walker
```sh
ros2 launch walker ros_bags.launch.py record_bag:=True
```
The recorded bag files will be stored in the results/bag_files directory of the walker package.


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
Run the walker node during playback to verify the recorded messages:
   ```sh
   ros2 run walker walker_node
   ```


## clang-tidy output
![clang-tidy](https://github.com/user-attachments/assets/28c6f674-a245-4a81-bdbe-bf7b5b28d188)



## clanglint output
![cpplint](https://github.com/user-attachments/assets/8b938b78-d0b2-45ed-a9bc-1d36fa7f60e5)




