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

while True:
  get_col()
  # get_values(1)