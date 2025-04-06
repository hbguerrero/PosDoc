Initial command to verify the perfiormance of miro_ros_pub_sub.ino:

From an Ubuntu 22 laptop:

Inside (Or corresponding workspace paste) ros2_ws enter following commands

  source/opt/ros/<rosDistro>/setup.bash
  source install/setup.bash 
  source ~/ros2_ws/install/local_setup.bash 

Verify ttyUSB or ttyACM ports:

  ls -la /dev | grep ttyUSB
or
  ls -la /dev | grep ttyACM

If required ports have no enough permissions:

  sudo usermod -a -G dialout <username>
  sudo chmod a+rw /dev/ttyUSB0         #or corresponding port


For a first time a micro_ros_agent has to be created, for that, we clone microRos2 following instructions in: 
https://www.youtube.com/watch?v=0R8VUPEkYhg&t=1704s

To run the micro_ros_agent:

  ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0  or corresponding ttyUSB
  //restart the board
  ros2 topic list
  ros2 topic echo /xiao_heartbeat
  ros2 topic pub /xiao_led_state std_msgs/Int32 "data: 0"
  ros2 topic pub /xiao_led_state std_msgs/Int32 "data: 1"
