# Bash aliases for ROS 2

# Basics
alias build_ws="colcon build --symlink-install"

create_ros2_pkg_c () {
    # Usage: create_ros2_pkg <new_package_name>
    ros2 pkg create $1 --build-type ament_cmake --dependencies rclcpp std_msgs
}

create_ros2_pkg_py () {
    # Usage: create_ros2_pkg <new_package_name>
    ros2 pkg create $1 --build-type ament_python --dependencies rclpy std_msgs
}

# To install missing dependencies
install_deps () {
    # Usage: Navigate to dev_ws and run install_deps
    rosdep install --from-paths src --ignore-src -r -y
}

compile_and_run () {
    # Usage: compile_and_run <package_name> <executable_name>
    colcon build --packages-select $1 --symlink-install && ros2 run $1 $2
}

kill_ros2 () {
    # Usage: kill_ros2
    killall -9 rviz2 gzserver gzclient
}

# To activate ROS environment 
alias source_ros="source /opt/ros/${ROS_DISTRO}/setup.bash"

# Activate the workspace
alias source_ws="source /home/dockeruser/dev_ws/install/setup.bash"

# Change directory to the workspace
alias cd_to_ws="cd /home/dockeruser/dev_ws"
alias cd_to_ws_src="cd /home/dockeruser/dev_ws/src"

# List all ROS 2 nodes, topics, services
alias list_nodes="ros2 node list"
alias list_topics="ros2 topic list"
alias list_services="ros2 service list"

echo_topic() {
    # Usage: echo_topic <topic_name>
    ros2 topic echo "$@"
}

# Show ROS 2 topic info
topic_info () {
    # Usage: topic_info <topic_name>
    ros2 topic info $1
}

# List all parameters for a ROS 2 node
list_params () {
    # Usage: list_params <node_name>
    ros2 param list $1
}

# Launch Riv
alias run_rviz="ros2 run rviz2 rviz2"

launch_with_rviz () {
    # Usage: launch_with_rviz <package_name> <launch_file>z
    ros2 launch $1 $2 & ros2 run rviz2 rviz2
}

# Launch Gazebo
alias run_gazebo_e="ros2 launch gazebo_ros empty_world.launch.py"
alias run_gazebo="ros2 launch gazebo_ros gazebo.launch.py"
