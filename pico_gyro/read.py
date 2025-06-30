import serial
import struct
import time

ser = serial.Serial('COM14', baudrate=115200)

angle = 0

ser.reset_input_buffer()
while True:
  sd = 'd'
  ser.write(sd.encode())
  # val = struct.unpack('<f', ser.read(4))[0]
  print(ser.readline().decode().strip())
  time.sleep(0.1)