from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch five camera publishers, each with its own camera index and topic name.
    """
    nodes = []

    # Create 5 publisher nodes for cameras 0 through 4
    for i in range(5):
        nodes.append(
            Node(
                package='camera_gui',
                executable='image_publisher',
                name=f'image_publisher_{i}',
                arguments=[str(i)],     # passes camera index to the node
                output='screen'
            )
        )

    return LaunchDescription(nodes)
