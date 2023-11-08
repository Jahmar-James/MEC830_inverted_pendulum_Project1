import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # Parameters
    force_value = DeclareLaunchArgument(
        'force_value', default_value='5.0', description='Disturbance force value for the pendulum'
    )

    # Node
    pendulum_disturbance_force_publisher_node = Node(
        package='inverted_pendulum_simulation',
        executable='force_publisher_node',
        name='pendulum_disturbance_force_publisher',
        parameters=[
            {'force_value': LaunchConfiguration('force_value')},
            {'FORCE_TOPIC': 'pendulum_disturbance_force'}
        ],
        output='screen'
    )

    return LaunchDescription([
        force_value,
        pendulum_disturbance_force_publisher_node
    ])
