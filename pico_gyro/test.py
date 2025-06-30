import serial
import time
import struct
from math import *
import numpy as np


def unwrap_angle(delta):
    """Unwrap angle to [-180, 180] range"""
    if delta > 180:
        delta -= 360
    elif delta < -180:
        delta += 360
    return delta

# Initial values
prev_heading = None
accumulated_angle = 0.0

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
  

  # print(calculate_heading(corrected[0], corrected[1]))
  # time.sleep(0.05)
  x, y, _ = corrected
  heading = degrees(atan2(y, x))
  if heading < 0:
    heading += 360

  # if prev_heading is not None:
  #   delta = unwrap_angle(heading - prev_heading)
  #   accumulated_angle += delta

  # prev_heading = heading
  print(f"Accumulated angle: {heading:.2f}°")





ser.close()
# f.close()