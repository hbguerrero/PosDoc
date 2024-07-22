#!/usr/bin/env python3

import os
import ydlidar
import time
import rospy

from std_msgs.msg import Float32

rospy.init_node("ydlidar_grades_publisher", anonymous = True)
pub = rospy.Publisher('ydlidarGrade', Float32, queue_size=5)
pub1 = rospy.Publisher('ydlidarRange', Float32, queue_size=5)
rate = rospy.Rate(1) # 1hz

while not rospy.is_shutdown():
#  rospy.loginfo()
  ydlidar.os_init();
  ports = ydlidar.lidarPortList();
  port = "/dev/ydlidar";
  for key, value in ports.items():
      port = value;
  laser = ydlidar.CYdLidar();
  laser.setlidaropt(ydlidar.LidarPropSerialPort, port);
  laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, 128000);
  laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE);
  laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL);
  laser.setlidaropt(ydlidar.LidarPropScanFrequency, 10.0);
  laser.setlidaropt(ydlidar.LidarPropSampleRate, 9);
  laser.setlidaropt(ydlidar.LidarPropSingleChannel, True);

  ret = laser.initialize();
  if ret:
      ret = laser.turnOn();
      scan = ydlidar.LaserScan();
      while ret and ydlidar.os_isOk() :
          r = laser.doProcessSimple(scan);
          if r:
              #print("Scan received[",scan.stamp,"]:",scan.points.size(),"ranges is [",1.0/scan.config.scan_time,"]Hz");
               for point in scan.points:
                 grades = point.angle*180/3.1416
                   
                 if grades >= 89 and grades <= 91:
                   print("angle:", grades, " range: ", point.range, "Intensity", point.intensity)
                   #time.sleep(0.25)
                   pub.publish(grades)
                   pub1.publish(point.range)
          else :
              print("Failed to get Lidar Data")
          time.sleep(0.05);
      laser.turnOff();
  laser.disconnecting();
