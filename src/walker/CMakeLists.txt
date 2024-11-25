cmake_minimum_required(VERSION 3.8)
project(walker)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(example_interfaces REQUIRED)

include_directories(include)

add_executable(walker_node src/walker.cpp)
ament_target_dependencies(walker_node 
                          rclcpp 
                          geometry_msgs 
                          sensor_msgs)



target_include_directories(walker_node PUBLIC
                          $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                          $<INSTALL_INTERFACE:include>
                        )


install(DIRECTORY
        launch
        worlds
        models
        urdf
        DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
        walker_node
        DESTINATION lib/${PROJECT_NAME})
  
ament_package()
