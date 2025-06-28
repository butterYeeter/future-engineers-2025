import serial
import struct
import time

ser = serial.Serial('COM14', baudrate=115200)

angle = 0

while True:
  # ser.flush()
  # val = struct.unpack('f'*3, ser.read(12))
  # print(ser.readline().decode())
  # print(ser.readline().decode().strip())
  v, dt = map(float, ser.readline().decode().strip().split(', '))
  # print(type(ser.readline()))
  
  angle += v * dt/10**6
  print(angle)