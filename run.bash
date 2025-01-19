colcon build
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run hello_robot_package human_det_node.py &
ros2 run hello_robot_package hello_robot_node