Este repositorio aloja las fuentes para la lectura de una imu (BNO055) a través de una placa arduino. Las lecturas de la imu se publican disponibilizandolas para el entorno ROS (Robotics Operational System).  El código para la arduino se encuentra en la fuente bno055ROS.ino. Nótese que se transmite apenas la medida de la orientación del plano x. La información desde la placa arduino es publicada en un tópico llamado info_back.

El proyecto en ROS considera un publicante que para el caso corresponde a la fuente ydLidar.py. Esta fuente lee las medidas conseguidas por el sensor LiDAr ydlidarX4 y las publica en los tópicos ydlidarGrade y ydlidarRange. ydlidarGrade publica el angulo en el cual logra la medida de distancia, en cuanto que ydlidarRange publica la medidad de distancia correspondiente.

En el marco del proyecto en ROS se configuró un subscriptor que corresponde a la fuente subscriberYDlidar.py. Es un subscriptor porque se sucribe a los tópicos info_back, ydlidarGrade y ydlidarRange.

Instrucciones para la instalación del sensor LiDar y su configuración para ROS, pueden seguirse en el siguiente enlace:
https://www.ydlidar.com/Public/upload/files/2024-05-07/YDLIDAR%20X4PRO%20Lidar%20User%20Manual%20V1.1(240507).pdf

la respectiva configuración para ROS de la biblioteca para el uso del YDlidarX4 en la carpeta adecuada, determinará la existencia de un paquete denominado ydlidar_ros_driver, el cual es una carpeta en la cual se debe contar con un subdirectorio para alojar los códigos fuente que publican los datos desde el sensor LiDar (ydLidar.py), así como el subscriptor (subscriberYDlidar.py).

La instalación de ROS se realiza sobre un computador con sistema operativo linux ubuntu 22. Instrucciones para la instalación de ROS pueden ser seguidas en el siguiente enlace: https://wiki.ros.org/noetic/Installation/Ubuntu

Las instrucciones para la instalación de bibliotecas y configurar la placa arduino, pueden seguirse en el siguiente enlace:
https://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup

En el siguiente enlace se encuentra un tutorial orientado al uso básico de la imu BNO055:
https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/processing-test

Los siguientes son instrucciones generales para correr la aplicación:

Contando con ROS y las bibliotecas para el LiDar y la Arduino debidamente instaladas, se deben seguir los siguientes pasos para correr la aplicación completa:

1. En el terminal de linux digite roscore.

2. Una vez ROS esta en ejecución se debe proceder a la lectura del mensaje publicado en el tópico desde la placa arduino a través del puerto USB:

      rosrun rosserial_python serial_node.py /dev/ttyACM0

3. Para proceder a realizar las lecturas del sensor LiDar y publicarlas en los tópicos:

      rosrun ydlidar_ros_driver ydLidar.py

4. Para leer los mensajes del LiDar y la IMU desde un subscriptor:

      rosrun ydlidar_ros_driver subcriberYDlidar.py








