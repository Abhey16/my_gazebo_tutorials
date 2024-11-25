import os
from datetime import datetime
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_unique_bag_path(base_path):
    """Generate a unique path for bag files by appending a counter if the directory exists."""
    counter = 0
    unique_path = base_path
    while os.path.exists(unique_path):
        counter += 1
        unique_path = f"{base_path}_{counter}"
    return unique_path


def generate_launch_description():
    # Paths and configurations
    package_name = 'walker'
    current_package_share = get_package_share_directory(package_name)
    gazebo_ros_share = get_package_share_directory('gazebo_ros')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    
    # World file path
    world_path = os.path.join(current_package_share, 'worlds', 'my_world.world')
    
    # Timestamp for bag file directory
    timestamp = datetime.now().strftime('%Y_%m_%d-%H_%M_%S')
    # workspace_src_dir = os.getenv('ROS_WORKSPACE', '.')
    workspace_src_dir = os.path.abspath(os.path.join(os.getenv('ROS_WORKSPACE', '.'), 'src'))
    results_dir = os.path.join(workspace_src_dir, package_name, 'results', 'bag_files')
    bag_path = generate_unique_bag_path(os.path.join(results_dir, timestamp))
    print(f"Bag file path: {bag_path}")  # Debugging print statement

    # Ensure the directory exists
    os.makedirs(os.path.dirname(bag_path), exist_ok=True)

    # Declare launch arguments
    record_bag_arg = DeclareLaunchArgument(
        'record_bag',
        default_value='True',
        description='Enable/disable ROS 2 bag recording'
    )

    # # Gazebo server and client
    # gzserver_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(gazebo_ros_share, 'launch', 'gzserver.launch.py')),
    #     launch_arguments={'world': world_path}.items()
    # )

    # gzclient_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(gazebo_ros_share, 'launch', 'gzclient.launch.py'))
    # )

    # Robot state publisher
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(current_package_share, 'launch', 'robot_state_publisher.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Spawning the robot
    spawn_robot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(current_package_share, 'launch', 'spawn_turtlebot3.launch.py')),
        launch_arguments={'x_pose': x_pose, 'y_pose': y_pose}.items()
    )

    # Walker node
    walker_node = Node(
        package=package_name,
        executable='walker_node',
        name='walker_node',
        output='screen',
    )

    # ROS 2 bag recording
    bag_record_cmd = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('record_bag')),
            
        cmd=[
            'ros2', 'bag', 'record', '--all', '-o', bag_path,
            '-x', '/camera/.*'  # Exclude all camera topics
        ],

        shell=True,
        output='screen'
    )

    # Launch description
    ld = LaunchDescription()

    # Add all actions and nodes
    ld.add_action(record_bag_arg)
    # ld.add_action(gzserver_cmd)
    # ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_robot_cmd)
    ld.add_action(walker_node)
    ld.add_action(bag_record_cmd)

    return ld
