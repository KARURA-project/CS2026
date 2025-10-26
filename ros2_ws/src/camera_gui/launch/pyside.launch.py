from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch five camera publishers and a single subscriber that displays all feeds.
    """
    nodes = []

    # Add five publisher nodes
    for i in range(5):
        nodes.append(
            Node(
                package='camera_gui',
                executable='image_publisher',
                name=f'image_publisher_{i}',
                arguments=[str(i)],
                output='screen'
            )
        )

    # Add one subscriber node that displays all feeds
    nodes.append(
        Node(
            package='camera_gui',
            executable='camera_with_pyside',
            name='camera_with_pyside',
            output='screen'
        )
    )

    return LaunchDescription(nodes)
