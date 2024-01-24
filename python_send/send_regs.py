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
reset_1 = "R0    0x2"
reset_1_b = str.encode(reset_1)
ser.write(reset_1_b)                # RESET = 1
time.sleep(0.2)                     # Wait 200 ms

reset_0 = "R0    0x0"
reset_0_b = str.encode(reset_0)
ser.write(reset_0_b)                # RESET = 0
time.sleep(0.02)
with open('HexRegisterValues.txt', 'rb') as file:
    for line in file:
        ser.write(line)
        time.sleep(0.02) #задержка в течение 0.02 секунд

file.close()
ser.close()  # Remember to close the connection when done
