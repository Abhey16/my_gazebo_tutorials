from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()

    my_pub_node = Node(
        package="beginner_tutorials",
        executable="my_node",
        # default value
        parameters=[
            {"publish_time":1}
        ],
        arguments = ['--x', '1', '--y', '1', '--z', '3', 
                     '--yaw', '1.57', '--pitch', '1.57', '--roll', '0', 
                     '--frame-id', 'world', 
                     '--child-frame-id', 'talk']
    )

    my_sub_node = Node(
        package="beginner_tutorials",
        executable="my_sub"
    )

    ld.add_action(my_pub_node)
    ld.add_action(my_sub_node)

    return ld