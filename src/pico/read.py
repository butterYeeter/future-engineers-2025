import serial
import struct
import time

ser = serial.Serial('COM10', baudrate=115200)

angle = 0

ser.reset_input_buffer()
def get_values(num):
  for i in range(num):
    ser.write('a'.encode())
    val = struct.unpack('<f', ser.read(4))[0]
    print(val)
    time.sleep(0.05)

# while True:
  # get_values(1)
#   sd = 'd'
#   ser.write(sd.encode())
#   # val = struct.unpack('<f', ser.read(4))[0]
#   # print(ser.readline().decode().strip())
#   time.sleep(0.1)

get_values(100)

calib = bytearray(2)
calib.append(ord('g'))
calib.append(100)
ser.write(calib)
time.sleep(10)
reset = b'r'
ser.write(reset)

get_values(100)