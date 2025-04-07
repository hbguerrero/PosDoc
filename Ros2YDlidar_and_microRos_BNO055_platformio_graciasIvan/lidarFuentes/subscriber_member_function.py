# Recibe el dato del bno (subscriptor), calcula error de orientación (eYaw)
# y lo publica en un tópico que llamamos u


import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Float32, Int32

global bno
bno = 9.9

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.publisher_ = self.create_publisher(Float32, 'u', 10)
        self.subscription = self.create_subscription(Float32, 'bno', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info("%f" % msg.data)
        if (msg.data >= 0.0 and msg.data <= 180.0):
           eYaw = 0.0 - msg.data
        if (msg.data >= 180.0 and msg.data <= 360.0):
           eYaw = 359.0 - msg.data
#       u = (msg.data)*2.0
        msg.data = eYaw
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MinimalSubscriber()
    rclpy.spin(node)
    #minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
