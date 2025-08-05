import rclpy
from rclpy.node import Node

import serial
import threading

from std_msgs.msg import Float32, Int32, String

class ESP32SerialBridge(Node):
    def __init__(self):
        super().__init__('esp32_serial_bridge')

        # 📟 Puerto serial
        self.serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)

        # 📨 Suscriptor para velocidad deseada
        self.create_subscription(String, 'velocidad_deseada', self.write_serial_callback, 10)

        # 📤 Publicadores
        self.omega1_pub = self.create_publisher(Float32, 'omega1', 10)
        self.omega2_pub = self.create_publisher(Float32, 'omega2', 10)
        self.imu_pub = self.create_publisher(Int32, 'imu_heading', 10)

        # 🧵 Hilo para leer el puerto serial
        thread = threading.Thread(target=self.serial_read_loop)
        thread.daemon = True
        thread.start()

        self.get_logger().info("Bridge ESP32–ROS2 listo.")

    def write_serial_callback(self, msg: String):
        try:
            comando = msg.data.strip()
            self.serial_port.write((comando + '\n').encode())
            self.get_logger().info(f'Enviado a ESP32: {comando}')
        except Exception as e:
            self.get_logger().error(f'Error enviando al ESP32: {e}')

    def serial_read_loop(self):
        while rclpy.ok():
            try:
                line = self.serial_port.readline().decode().strip()
                if line.startswith('>omega1:'):
                    value = float(line.replace('>omega1:', ''))
                    self.omega1_pub.publish(Float32(data=value))

                elif line.startswith('>omega2:'):
                    value = float(line.replace('>omega2:', ''))
                    self.omega2_pub.publish(Float32(data=value))

                elif line.startswith('>imu:'):
                    value = int(line.replace('>imu:', ''))
                    self.imu_pub.publish(Int32(data=value))

            except Exception as e:
                self.get_logger().warn(f'Error leyendo serial: {e}')


def main():
    rclpy.init()
    node = ESP32SerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
