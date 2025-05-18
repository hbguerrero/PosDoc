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

    python3 test2.py

##################################################################################################################

2. AL respecto del BNO055

Para el caso del bno055, para ser usado desde microRos, se desarrolló un proyecto usando platformIo configurado como ua extensión de visual studio code

Consecuentemente:

2.1 Se debe crear un proyecto en platformIo para el ambiente de Arduino, teniendo cuidado de seleccionar la placa esp32 correcta. Para nuesro caso ya contamos con el archivo main.cpp, este código se debe cargar en el archivo main.cpp del proyecto.

2.2 El archivo platformio.ini debe contener las lineas que estan en este repositorio, en el archivo con ese mismo nombre.

2.3 Para dar cumpliento a la exixtencia de la dependencia "adafruit/Adafruit BNO055@^1.6.4" en el archivo platformio.ini:

    a) Debe descargarse el archivo de este repositorio "BNO055_Reader_Arduino_Platformio.zip" y descomprimirlo, con eso es suficiente


Referencias adicionales:

https://www.youtube.com/watch?v=0R8VUPEkYhg&t=1804s
https://www.linkedin.com/pulse/micro-ros-esp32-ibrahim-bin-mansur-kmzwf/


###################################################################################################################

Ojo: Documentar los pasos para configurar uRos y correrlo


