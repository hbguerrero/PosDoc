import serial, time
#import serial #Importa a biblioteca
import sys, termios, tty, os, time
import time

#arduino = serial.Serial("/dev/ttyACM0", 9600)
#time.sleep(2)
#arduino.write(b'9')
#arduino.close()

def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

button_delay = 0.05

while True: #Loop para a conexão com o Arduino
    
    try:  #Tenta se conectar, se conseguir, o loop se encerra
        arduino = serial.Serial('/dev/ttyACM0', 9600)
        print('Arduino conectado')
        break

    except:
        pass

while True: #Loop principal
    #cmd = input('Digite "l" para ligar e "d" para desligar. ') #Pergunta ao usuário se ele deseja ligar ou desligar o led
    cmd = getch()
    if (cmd == "5"):
        arduino.write('5'.encode())
        print("Stop!")
        exit(0)
        
    elif cmd == '2': #Se a resposta for "l", ele envia este comando ao Arduino
        arduino.write('2'.encode())
        print(cmd)
    elif cmd == '3': #Senão, envia o "d"
        arduino.write('3'.encode())
        print(cmd)
    elif cmd == '1': #Senão, envia o "d"
        arduino.write('1'.encode())
        print(cmd)
    elif cmd == '8': #Senão, envia o "d"
        arduino.write('8'.encode())
        print(cmd)

arduino.flush() #Limpa a comunicação
