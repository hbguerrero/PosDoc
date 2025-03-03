import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Int32, Float32

import os
from . import ydlidar

import time
import math


ydlidar.os_init()
ports = ydlidar.lidarPortList()
port = "/dev/ydlidar"

lx = 0
sumatoria = 0
promedio = 0
contador = 0

for key, value in ports.items():
    port = value
laser = ydlidar.CYdLidar()
laser.setlidaropt(ydlidar.LidarPropSerialPort, port)
laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, 115200)
laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE)
laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL)
laser.setlidaropt(ydlidar.LidarPropScanFrequency, 10.0)
laser.setlidaropt(ydlidar.LidarPropSampleRate, 9)
laser.setlidaropt(ydlidar.LidarPropSingleChannel, True)


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Float32, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0.0
        

    def timer_callback(self):
        msg = Float32()
        
        self.get_logger().info('Publishing: "%f"' % msg.data)
        #self.i = promedio
        
        ret = laser.initialize()

        if ret:
            ret = laser.turnOn()
            scan = ydlidar.LaserScan()
            while ret and ydlidar.os_isOk():
                r = laser.doProcessSimple(scan)
                sumatoria = 0
                contador = 1
                promedio = 0.0
                if r:
                    # print("Scan received[",scan.stamp,"]:",scan.points.size(),"ranges is [",1.0/scan.config.scan_time,"]Hz");
                    for point in scan.points:
                        if (point.angle < 1.745) and (point.angle > 1.571):
                            #                   if point.angle > math.pi/2:
                            if (point.range > 0) and (point.range < 1.8):
                                lx = point.range * \
                                    math.cos(point.angle - math.pi/2)
                                sumatoria = sumatoria + lx
                                contador = contador + 1
                                self.i = sumatoria/contador
                                msg.data = self.i
                                self.publisher_.publish(msg)
                                #print(promedio)
            #                    print("angle:", point.angle, " range: ", point.range)#
            #                    print("angle:", point.angle, " lx: ", lx)
                                # print("angle:", point.angle, " range: ", lx)
            #                   time.sleep(1)
                else:
                    print("Failed to get Lidar Data")
                time.sleep(0.05)
            laser.turnOff()
        laser.disconnecting()


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
