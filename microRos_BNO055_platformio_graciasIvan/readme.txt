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






