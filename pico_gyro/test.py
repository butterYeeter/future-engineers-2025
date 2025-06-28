import serial
import time
import struct
from math import *
import numpy as np

ser = serial.Serial('COM14', baudrate=115200)
# f = open("test.txt", 'w')

# t1 = time.time();

# while time.time() - t1 < 30:
#   ser.reset_input_buffer()
#   line = ser.readline().decode()
#   f.write(line)
#   # print("Hello");

bias = np.array([-43.924241, 12.448210, -73.133957])
scale = np.array([[1.096529, -0.033064, -0.055229],
                  [-0.033064, 0.989653, -0.120042],
                  [-0.055229, -0.120042, 0.819875]])



def calculate_heading(x, y):
    heading_rad = atan2(y, x)
    heading_deg = degrees(heading_rad)
    if heading_deg < 0:
        heading_deg += 360
    return heading_deg


while True:
  ser.reset_input_buffer()
  vals = struct.unpack('<fff', ser.read(12))

  raw = np.array(vals)
  corrected = scale @ (raw - bias)
  # atan2()
  

  print(calculate_heading(corrected[0], corrected[1]))
  time.sleep(0.05)





ser.close()
# f.close()