import serial

ser = serial.Serial('/dev/ttyACM0',9600)
light_meter = 0


while True:
 read_serial=ser.readline()
 light_meter = int(read_serial)
 print(light_meter)