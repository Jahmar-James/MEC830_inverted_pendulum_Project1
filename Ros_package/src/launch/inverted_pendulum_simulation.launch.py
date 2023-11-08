import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir, LaunchConfiguration

from ament_index_python.packages import get_package_share_directory
gazebo_ros_share_dir = get_package_share_directory('gazebo_ros')

def generate_launch_description():
    
    # Arguments
    use_gui = DeclareLaunchArgument(
        'gui', default_value='true', description='Set to "false" to run Gazebo headless'
    )

    use_server = DeclareLaunchArgument(
        'server', default_value='true', description='Set to "false" not to run gzserver'
    )

    # Nodes
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        arguments=['src/inverted_pendulum_simulation/urdf/inverted_pendulum.urdf']
    )

    rotary_encoder_node = Node(
        package='inverted_pendulum_simulation',
        executable='rotary_encoder_node'
    )

    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/robot_description', '-entity', 'my_robot']
    )

    # Include Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_ros_share_dir, '/launch/gazebo.launch.py']),
        launch_arguments={
            'gui': LaunchConfiguration('gui'),
            'server': LaunchConfiguration('server')
        }.items()
    )

    return LaunchDescription([
        use_gui,
        use_server,
        gazebo_launch,
        robot_state_publisher_node,
        rotary_encoder_node,
        spawn_entity_node
    ])
