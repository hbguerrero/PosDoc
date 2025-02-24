Initial command to verify the perfiormance of miro_ros_pub_sub.ino:

From a Ubuntu 22 laptop:

source ~/ros2_ws/install/local_setup.bash 
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
//restar the board
ros2 topic list
ros2 topic pub /xiao_led_state std_msgs/Int32 "data: 0"
ros2 topic pub /xiao_led_state std_msgs/Int32 "data: 1"
