import serial #Importa a biblioteca
import sys, termios, tty, os, time
import time

def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

button_delay = 0.2

while True: #Loop para a conexão com o Arduino
    
    try:  #Tenta se conectar, se conseguir, o loop se encerra
        arduino = serial.Serial('/dev/ttyUSB0', 9600)
        print('Arduino conectado')
        break

    except:
        pass

while True: #Loop principal
    #cmd = input('Digite "l" para ligar e "d" para desligar. ') #Pergunta ao usuário se ele deseja ligar ou desligar o led
    cmd = getch()
    if (cmd == "p"):
        arduino.write('p'.encode())
        print("Stop!")
        exit(0)
        
    elif cmd == 'w': #Se a resposta for "l", ele envia este comando ao Arduino
        arduino.write('w'.encode())
        print(cmd)
    elif cmd == 's': #Senão, envia o "d"
        arduino.write('s'.encode())
        print(cmd)
    elif cmd == 'd': #Senão, envia o "d"
        arduino.write('d'.encode())
        print(cmd)
    elif cmd == 'a': #Senão, envia o "d"
        arduino.write('a'.encode())
        print(cmd)

arduino.flush() #Limpa a comunicação  