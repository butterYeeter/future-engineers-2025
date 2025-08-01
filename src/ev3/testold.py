#!/usr/bin/env pybricks-micropython
from pixy import Pixy
from usb import USB
from color import ColorSensor
from gyro import Gyro
from pid import PIDController
from pybricks.ev3devices import Motor
from pybricks.parameters import Port
from calib import calib_steering
from util import is_blue

steer_motor = Motor(Port.C)
drive_motor = Motor(Port.B)

pid = PIDController(target=316//2, Kp=0.5, output_limit=30)
pixy = Pixy(Port.S2, 0x54)
# pixy.set_lamp(1)

usb = USB()
color = ColorSensor(usb, (0.43243244, 0.32432431, 0.21621622))
gyro = Gyro(usb)
target_angle = gyro.get_angle()
gpid = PIDController(target_angle, 2.2, 0.02, 1.5, 35, 5)

calib_steering(steer_motor)
drive_motor.dc(70)

b = pixy.get_largest_signiture()
area = b["w"] * b["h"]
print("Area is {}".format(area))


ignore = 0
k_gyro = 0
k_pixy = 1
turning = False
change = 0
while True: 
  block = pixy.get_largest_signiture()
  if block["type"] == 1:
    pid.target = 292
    change = -30
  elif block["type"] == 2:
    pid.target = 65
    change = 30

  if ignore <= 0:
    current_angle = gyro.get_angle()
    correction = -k_pixy * pid.loop(block["cx"]) + k_gyro * gpid.loop(current_angle)
    steer_motor.track_target(correction)

    if abs(current_angle - gpid.target) <= 10:
      turning = False
      drive_motor.dc(70)
  else:
    correction = gpid.loop(gyro.get_angle())
    steer_motor.track_target(correction)
    ignore -= 1
    if ignore == 10:
      gpid.target -= change
      change = 0

  area = block["w"] * block["h"]
  passing_block = False
  if pid.target == 65:
    passing_block = block["cx"] < pid.target
  elif pid.target == 292:
    passing_block = block["cx"] > pid.target
  
  if area > 2100 and passing_block and ignore == 0:
    # drive_motor.brake()
    gpid.target += change
    ignore = 20
  
  if is_blue(color.get_color_corrected()):
    # drive_motor.brake()
    if turning == True:
      continue

    turning = True
    drive_motor.dc(100)
    gpid.target += 90
    k_pixy = 0
    k_gyro = 1
    # target_angle += 90
    # gpid.target = target_angle
    
    # drive_motor.dc(-60)
    # cur_ang = gyro.get_angle()
    # while abs(cur_ang - gpid.target) > 5:
    #   corr = -gpid.loop(cur_ang)
    #   steer_motor.track_target(corr)
    # drive_motor.brake()
    # drive_motor.dc(60)
  

