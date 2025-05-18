1. AL respecto del LIDAR YDLidar X2 o X4:

Iván creó un script que permite crear una biblioteca para su uso, se deben seguir los siguientes pasos:

1.1 Se debe descargar el archivo "compile-SDKLidar-python-lib(1).sh"

1.2 Desde la carpeta donde se descargó el archivo previamente mecionado, se debe otirgar permisos de ejecución de ese  archivo, para ello se debe correr el siguiente comando:

    chmod +x compile-SDKLidar-python-lib.sh

1.3 Una vez se cueta con ele ejecutable, ejecutamos:

    ./compile-SDKLidar-python-lib.sh

1.5 Lo anterior permite instalar la biblioteca usando el siguiente comando:

    pip3 install ydlidar-1.2.4-cp312-cp312-linux_x86_64.whl --break-system-packages

1.6 Se comparte un programa de prueba "test2.py" que se puede correr con el siguiente comando

    test2.py

##################################################################################################################



Gracias a Iván, quien desde un princicpio me indicó usar platformio...

Se realizó la lectura y publicación de un BNO055 usando microRos y Ros2 humble en particular en un portatil x64 con linuc¿x ubuntu 22

Inicialmente se sigió el tutorial en el siguiente enlace:  https://www.youtube.com/watch?v=0R8VUPEkYhg&t=1804s

Posteriormente se decidió trabajar desde visual studio code usando platformio.

Para configurar ros2, buscando mejorar la frecuencia de la publicación desde un esp32, se sigió el siguiente tutorial: https://www.linkedin.com/pulse/micro-ros-esp32-ibrahim-bin-mansur-kmzwf/

En la carpeta "fuentes" se encuentra el main.cpp y platformio.ini.
En el platformio.ini "adafruit/Adafruit BNO055@^1.6.4" corresponde a una biblioteca que me pasó Iván. Lo demáss es del tutorial en linkedin.
En el platformio.ini, nótese que la placa es para nuestro esp32, aa diferenia del tutorial en linkedin.

En el main.cpp hay un publicante que corresponde al esp32 publicando lo de la imu.
En el main.cpp hay un subscriptor que corresponde al esp32 recibiendo mensajes para encender unos leds.
