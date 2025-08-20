#!/usr/bin/env pybricks-micropython
from pixy import Pixy
from color import ColorSensor
from gyro import Gyro
from pid import PIDController
from pybricks.ev3devices import Motor
from pybricks.iodevices import UARTDevice
from pybricks.parameters import Port
from calib import calib_steering
from util import is_blue
from pybricks.tools import DataLog

steer_motor = Motor(Port.D)
drive_motor = Motor(Port.B)

pid = PIDController(target=316//2, Kp=0.1, Kd=0.05, output_limit=30)
pixy = Pixy(Port.S2, 0x54)

pico_uart = UARTDevice(Port.S3, 115200)
pico_uart.clear()
color = ColorSensor(pico_uart, (0.43243244, 0.32432431, 0.21621622))
gyro = Gyro(pico_uart)
target_angle = gyro.get_angle()
gpid = PIDController(target_angle, 2.2, 0.02, 1.5, 32, 5)

calib_steering(steer_motor)

b = pixy.get_largest_signiture()
area = b["w"] * b["h"]
print("Area is {}".format(area))


log = DataLog("", name="../log", timestamp=False)

drive_motor.run(1500)
dodging = False
dodge_obstacle = 0
change = 30
counter = 0
correction = 0
  
while True:
  block = pixy.get_largest_signiture()
  angle = gyro.get_angle()

  if block["type"] != 1 and block["type"] != 2:
    correction = gpid.loop(angle)
    print("g")
  else:
    correction = -pid.loop(block["cx"])
    print("c")

  # correction = gpid.loop(angle)
  steer_motor.track_target(correction)

  area = block["w"] * block["h"]
  if area > 3500:
    # drive_motor.brake()
    if block["type"] == 1:
      dodging = True
      change = -30
    if block["type"] == 2:
      dodging = True
      change = 30

  # if is_blue(color.get_color_corrected()):
  #   gpid.target += 90

  if dodging:
    dodging = False
    counter = 70
    prevtarg = gpid.target
    gpid.target = angle + change
    while counter > 0:
      c = color.get_color_corrected()
      # if is_blue(c):
      print(counter)
        
         

      counter -= 1
      if counter == 30:
        gpid.target = prevtarg
      
      correction = gpid.loop(gyro.get_angle())
      steer_motor.track_target(correction)
    
    # drive_motor.brake()
    # break
      


