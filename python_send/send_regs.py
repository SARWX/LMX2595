import serial.tools.list_ports
import serial
import time

connect = 0
while connect == 0:
    ports = serial.tools.list_ports.comports()
    for port in ports:
        print(port.device)

    COM_PORT = input("Введите номер COM порта из перечисленных выше или неправильное значение для повторного вывода COM портов: ")

    for port in ports:
        if (COM_PORT == port.device) :
            connect = 1
   
print(COM_PORT)
port = COM_PORT
baudrate = 9600  

ser = serial.Serial(port, baudrate)
with open('reg.txt', 'r') as file:
    lines = file.readlines()

for line in lines:
    ser.write(line)
    time.sleep(0.2) #задержка в течение 0.02 секунд

file.close()
ser.close()  # Remember to close the connection when done
