import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Float32

import os
import ydlidar
import time
import math
import csv

ydlidar.os_init()
ports = ydlidar.lidarPortList()
port = "/dev/ydlidar"

lx = 0
sumatoriaLeft = 0
promedioLeft = 0
contadorLeft = 0
sumatoriaRight = 0
promedioRight = 0
contadorRight = 0
yaw = 0.0
vuelta = 0

BWHP = 10
DEG2RAD = 0.0174533
PIMED = 1.5708


for key, value in ports.items():
    port = value
laser = ydlidar.CYdLidar()
laser.setlidaropt(ydlidar.LidarPropSerialPort, port)
laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, 112500)
laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE)
laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL)
laser.setlidaropt(ydlidar.LidarPropScanFrequency, 10.0)
laser.setlidaropt(ydlidar.LidarPropSampleRate, 9)
laser.setlidaropt(ydlidar.LidarPropSingleChannel, True)


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')

        self.subscription = self.create_subscription(Float32, 'bno', self.listener_callback, 10)
        self.publisher_Left = self.create_publisher(Float32, 'lidarLeft', 10)
        self.publisher_Right = self.create_publisher(Float32, 'lidarRight', 10)

        """
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        """
        self.i = 0.0
        self.j = 0.0
        
    def listener_callback(self, msg):

        ret = laser.initialize()

        if ret:
            ret = laser.turnOn()
            scan = ydlidar.LaserScan()
            if ret and ydlidar.os_isOk():
                r = laser.doProcessSimple(scan)
                sumatoriaLeft = 0
                contadorLeft = 0.0
                promedioLeft = 0.0
                
                sumatoriaRight = 0
                contadorRight = 0.0
                promedioRight = 0.0
                
                yaw = msg.data
                if msg.data == 360.0:
                    self.rads = 0.0
                else:
                    self.rads = (msg.data)*math.pi/180

                if 0 <= self.rads and self.rads < math.pi/2:
                    self.eYaw = 0.0 - self.rads
                elif self.rads < 2*(math.pi) and self.rads > 3*(math.pi/2):
                    self.eYaw = 2*(math.pi) - self.rads
                

                if r:
                    with open('LidarReadings1.csv', 'a') as csvfile:
                        writer = csv.writer(csvfile)
                        row = []
                           #print("Scan received[",scan.stamp,"]:",scan.points.size(),"ranges is [",1.0/scan.config.scan_time,"]Hz");
                        for point in scan.points:
                           #if point.angle < ((PIMED + ((BWHP*DEG2RAD)/2)) + self.eYaw) and (point.angle > ((PIMED - ((BWHP*DEG2RAD)/2)) + self.eYaw)):
                           if (point.angle < (-0.349 + self.eYaw)) and (point.angle > (-2.792 + self.eYaw)):
                              
                              if point.range > 0.0:
                                 lx = point.range*math.cos(point.angle  - (math.pi/2 + self.eYaw))  ## Aqui SI disloca el cono, solo usu eYaw para la proyección
                                 if (lx > -2):
                                    sumatoriaLeft = sumatoriaLeft + lx
                                    contadorLeft = contadorLeft + 1
                                    self.i = sumatoriaLeft/contadorLeft
                            
                           if (point.angle < (2.792 + self.eYaw)) and (point.angle > (0.349 + self.eYaw)):
                              #reading = ""
                              if point.range > 0.0:
                                 lx = point.range*math.cos(point.angle  - (math.pi/2 + self.eYaw))  ## Aqui SI disloca el cono, solo usu eYaw para la proyección
                                 if (lx < 2):
                                    sumatoriaRight = sumatoriaRight + lx
                                    contadorRight = contadorRight + 1
                                    self.j = sumatoriaRight/contadorRight
                                    
                            #self.i = 0.0
                           #reading += str(point.range)+"@"+str(point.angle)+"@"+str(self.eYaw)+"@"+str(self.i)+"@"+str(self.j)
                              #reading += str(point.range)+"@"+str(point.angle)+"@"+str(yaw)
                              #row.append(reading)
                              if point.angle < 0:
                                  theta = math.degrees(point.angle) + 360
                              else:
                                  theta = math.degrees(point.angle)      
                              
                              
                        #writer.writerow([vuelta,point.range, theta, yaw])
                        
                        
                                    #print(promedio)
                        #             print("angle:", point.angle, " range: ", point.range)#
                #                    print("angle:", point.angle, " lx: ", lx)
                                    # print("angle:", point.angle, " range: ", lx)
                #                   time.sleep(1)
                        msg.data = self.i
                        #print(self.i, self.eYaw)
                        self.publisher_Left.publish(msg)
                        msg.data = self.j
                        #print(self.i, self.eYaw)
                        self.publisher_Right.publish(msg)
                                                               
                
                else:
                    print("Failed to get Lidar Data")
                time.sleep(0.05)
                
                
def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()
    laser.turnOff()
    laser.disconnecting()


if __name__ == '__main__':
    main()