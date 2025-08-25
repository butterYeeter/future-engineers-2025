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

def get_col():
  ser.write('c'.encode())
  val = struct.unpack("<fff", ser.read(12))
  print(f"{val[0]:.8f}, {val[1]:.8f}, {val[2]:.8f}")
  time.sleep(0.1)

def get_det():
  ser.write('d'.encode())
  # print(ser.readline())
  data = ser.read(4)
  val = struct.unpack("<i", data)[0]
  ser.write('e'.encode())
  print(f"DATA: {data}, DETECTED COLOR: {val}")



while True:
  # get_col()
  get_det()
  time.sleep(0.5)
  # get_values(1)