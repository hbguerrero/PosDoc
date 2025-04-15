# Recibe el dato del bno (subscriptor), calcula error de orientación (eYaw)
# y lo publica en un tópico que llamamos u

import rclpy
from rclpy.node import Node
import math

from std_msgs.msg import String, Float32, Int32

#global bno
#bno = 9.9

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.publisher_ = self.create_publisher(Float32, 'u', 10)
        self.subscription = self.create_subscription(Float32, 'bno', self.listener_callback, 10)
        self.subscription1 = self.create_subscription(Float32, 'lidar', self.listener_callback1, 10)
        #self.subscription  # prevent unused variable warning
        self.eYaw = 0.0

    def listener_callback(self, msg):
        self.get_logger().info("%f" % msg.data)
        
        if msg.data == 360.0:
           self.rads = 0.0
        else:
            self.rads = (msg.data)*math.pi/180

        if 0 <= self.rads and self.rads < math.pi/2:
            self.eYaw = 0.0 - self.rads
        elif self.rads < 2*(math.pi) and self.rads > 3*(math.pi/2):
            self.eYaw = 2*(math.pi) - self.rads
        print("eYaw: ", self.eYaw)
        #msg.data = self.eYaw
        #self.publisher_.publish(msg)  
        
    def listener_callback1(self, msg):
        #self.get_logger().info("%f" % msg.data)
        self.angleError = self.eYaw 
        self.lidar = msg.data
        self.lx = self.lidar*math.cos(self.eYaw)
        print("lx: ", self.lx)

        if self.lx <= 0.8 and self.lx >= 0.5:
            self.controlAction = 0.0
        elif self.lx > 0.8:
            self.controlAction = 5.0
        else:
            self.controlAction = -5.0
          
        msg.data = self.controlAction

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MinimalSubscriber()
    rclpy.spin(node)
    #minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()