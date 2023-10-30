import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import xacro


from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    # Path to your URDF file
    
    pkg_name = 'inverted_pendulum_simulation'
    file_subpath = 'urdf/inverted_pendulum.xacro'
    
    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()
    
    # Node to publish robot state
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw}] # add other parameters here if required
    )
    
    # Rotary Encoder Node
    node_encoder = Node(
        package='inverted_pendulum_simulation',
        executable='rotary_encoder_node',
        output='screen',
    )
    
    gazebo_launch_node = Node(
        package='gazebo_ros',
        executable='gazebo',
        output='screen',
        arguments=['--verbose', '-s', 'libgazebo_ros_init.so']
    )

    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_robot'],
        output='screen',
        parameters=[{'robot_description': LaunchConfiguration('robot_description')}]
    )
    
    
    urdf = os.path.join(get_package_share_directory('inverted_pendulum_simulation'), 'urdf', 'inverted_pendulum.urdf') 
    print("Loading URDF from:", urdf)

    return LaunchDescription([
        # Declare the robot_description as a launch configuration
        DeclareLaunchArgument(
            'robot_description',
            default_value=robot_description_raw,
            description='Robot description from xacro processed URDF'
        ),
        gazebo_launch_node,
        # Start the robot state publisher node
        node_robot_state_publisher,

        # Add the rotary encoder node
        node_encoder,
        
      
        
        spawn_entity_node
    ])
