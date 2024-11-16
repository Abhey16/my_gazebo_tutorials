cmake_minimum_required(VERSION 3.8)
project(beginner_tutorials)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(catch_ros2 REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)


include_directories(include)

add_executable(my_node src/my_node.cpp)
ament_target_dependencies(my_node 
                          rclcpp 
                          example_interfaces
                          geometry_msgs
                          std_srvs
                          tf2
                          tf2_ros
                          )

add_executable(my_sub src/my_sub.cpp)
ament_target_dependencies(my_sub 
                          rclcpp 
                          example_interfaces)

add_executable(my_node_test test/my_node_test.cpp)
# The link libraries call this node with "catch_ros2::catch_ros2_with_node_main"
# to get the default integration test node main function
target_link_libraries(my_node_test
  catch_ros2::catch_ros2_with_node_main
  )
ament_target_dependencies(my_node_test
                          rclcpp 
                          std_srvs 
                          std_msgs
                          )

target_include_directories(my_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# This CMake function allows the integration test to be run
# when running "colcon test".
catch_ros2_add_integration_test (ExampleIntegration_TestYAML
  LAUNCH_FILE integration_test.launch.yaml  # example in YAML
  )

install(TARGETS
  my_node
  my_sub
  my_node_test
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME}
)

ament_package()
