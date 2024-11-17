from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from datetime import datetime
import os
import ament_index_python.packages  # Import to get package share directory


def generate_unique_bag_path(base_path):
    """Generate a unique path for bag files by appending a number if the directory exists."""
    counter = 0
    unique_path = base_path
    while os.path.exists(unique_path):
        counter += 1
        unique_path = f"{base_path}_{counter}"
    return unique_path


def generate_launch_description():
    # Create a timestamp for the directory name
    timestamp = datetime.now().strftime('%Y_%m_%d-%H_%M_%S')

    # Declare the argument for enabling or disabling bag recording
    bag_recording_arg = DeclareLaunchArgument(
        'record_bag', 
        default_value='True',
        description='Enable/Disable bag recording',
        choices=['True', 'False']
    )

    # Get the absolute path of the workspace src directory
    workspace_src_dir = os.path.join(os.path.abspath(os.getenv('ROS_WORKSPACE', '.')), 'src')

    # Define the relative path for saving bag files in 'src/beginner_tutorials/results/bag_files'
    relative_bag_path = os.path.join(workspace_src_dir, 'beginner_tutorials', 'results', 'bag_files', timestamp)

    # Generate a unique directory for the bag files (to avoid overwriting)
    bag_path = generate_unique_bag_path(relative_bag_path)
    print(f"Bag file path: {bag_path}")  # Debugging print statement

    # Define the rosbag recording process
    bag_record_cmd = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('record_bag')),
        cmd=[
            'ros2', 'bag', 'record', '-o', bag_path, '/my_topic',
            '--compression-mode', 'file',
            '--compression-format', 'zstd'
        ],
        shell=True,
        output='screen'
    )

    # Define the publisher node
    publisher_node = Node(
        package="beginner_tutorials",
        executable="my_node",
        name='my_publisher',
        parameters=[
            {"publish_time": 1}
        ],
        arguments=['talk', '1', '1', '3', '0', '1.57', '1.57'],
        output='screen'
    )

    return LaunchDescription([
        # Launch Arguments
        bag_recording_arg,
        
        # Nodes
        bag_record_cmd,
        publisher_node
    ])
