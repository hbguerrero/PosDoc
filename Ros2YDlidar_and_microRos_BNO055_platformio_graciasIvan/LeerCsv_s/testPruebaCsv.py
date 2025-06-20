import os
import ydlidar
import time
import math
import csv
import math

ydlidar.os_init();
ports = ydlidar.lidarPortList();
port = "/dev/ydlidar";
#port = "/dev/ttyUSB1";

lx = 0
sumatoria = 0
promedio = 0
contador = 0
vuelta = 0

for key, value in ports.items():
    port = value;
laser = ydlidar.CYdLidar();
laser.setlidaropt(ydlidar.LidarPropSerialPort, port);
laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, 115200);
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
        sumatoria = 0
        contador = 1        
        if r:
            with open('LidarReadings.csv', 'a') as csvfile:
                writer = csv.writer(csvfile)
                row = []
                                    
            #print("Scan received[",scan.stamp,"]:",scan.points.size(),"ranges is [",1.0/scan.config.scan_time,"]Hz");
                for point in scan.points:
                    reading = ""
                    #if (point.angle < 1.745) and  (point.angle > 1.571):  
                        #if (point.range > 0) and (point.range < 1.8):
                            # lx = point.range*math.cos(point.angle - math.pi/2)
                            # sumatoria = sumatoria + lx
                            # contador = contador + 1
                            # promedio = sumatoria/contador
                            # print(promedio)
#                    print("angle:", point.angle, " range: ", point.range)#
#                    print("angle:", point.angle, " lx: ", lx)
                     #print("angle:", point.angle, " range: ", lx)
#                   time.sleep(1)

                    #reading += str(point.range) + "@" + str(point.angle)
                    if point.angle < 0:
                        theta = math.degrees(point.angle) + 360
                    else:
                        theta = math.degrees(point.angle)
                    #reading += str(vuelta) + "@" + str(point.range) + "@" + str(theta)
                    #row.append(reading)
                    #writer.writerow(row)
                    writer.writerow([vuelta, point.range, theta])   
                
                vuelta = vuelta + 1

        else :
            print("Failed to get Lidar Data")
        time.sleep(0.05);
    laser.turnOff();
laser.disconnecting();