#!/usr/bin/env pybricks-micropython
from usb import USB
from pixy import Pixy
from color import ColorSensor
from gyro import Gyro
from pid import PIDController
from pybricks.ev3devices import Motor
from pybricks.iodevices import UARTDevice
from pybricks.parameters import Port, Stop
from util import *
from pybricks.tools import DataLog


LEFT = 90
RIGHT = -90
SPEED = 800


steer_motor = Motor(Port.D)
drive_motor = Motor(Port.B)

pid = PIDController(target=316//2, Kp=0.25, Kd=0.05, output_limit=40)
pixy = Pixy(Port.S2, 0x54)

usb = USB()
pico_uart = UARTDevice(Port.S4, 115200)
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

drive_motor.run(SPEED)
dodging = False
dodge_obstacle = 0
change = 30
counter = 0
correction = 0
previous_obstacle = 0
lt = False

def l_turn(direction):
  distance = distance_to_angle(500) + drive_motor.angle()
  while drive_motor.angle() < distance:
      correction = gpid.loop(gyro.get_angle())
      steer_motor.track_target(correction)

  drive_motor.run(-SPEED)
  gpid.target += direction
  angle = gyro.get_angle()
  while abs(gpid.target - angle) > 3:
      angle = gyro.get_angle()
      correction = -gpid.loop(angle)
      steer_motor.track_target(correction)

  drive_motor.run(SPEED)
    
  
while True:
  block = pixy.get_largest_signiture()
  angle = gyro.get_angle()

  if block["type"] != 1 and block["type"] != 2:
    correction = gpid.loop(angle)
    # print("g")
  else:
    correction = -pid.loop(block["cx"])
    # print("c")

  # correction = gpid.loop(angle)
  steer_motor.track_target(correction)

  area = block["w"] * block["h"]
  if area > 3000:
    # drive_motor.brake()
    if block["type"] == 1:
      dodging = True
      change = -35
    if block["type"] == 2:
      dodging = True
      change = 35

  # if is_blue(color.get_color_corrected()):
  #   gpid.target += 90
  if lt > 0:
    l_turn(lt)
    previous_obstacle = 0
    lt = 0
  # detected_col = color.get_detected()
  # c = color.get_color_corrected()
  # detected_col = get_color(c)
  # print(detected_col)
  # if detected_col == ORANGE:
  #   gpid.target = prevtarg + RIGHT
  #   print("ORANGEEGEEEE")
  c = color.get_color_corrected()
  detected_col = get_color(c)
  if detected_col == BLUE:
      print("Turning left...")
      gpid.target += LEFT
      # self.color.reset_detected()
  elif detected_col == ORANGE:
      print("Turning RIGHT")
      gpid.target += RIGHT

  if dodging:
    dodging = False
    previous_obstacle = block["type"]
    # 420
    distance = distance_to_angle(350) + drive_motor.angle()
    counter = 70
    prevtarg = gpid.target
    # gpid.target = angle + change
    gpid.target += change
    while drive_motor.angle() < distance:
      # counter -= 1
      if counter == 30:
        gpid.target = prevtarg
      
      correction = gpid.loop(gyro.get_angle())
      steer_motor.track_target(correction)

    distance = distance_to_angle(100) + drive_motor.angle()
    gpid.target = prevtarg
    while drive_motor.angle() < distance:
      # print("TEST")
      correction = gpid.loop(gyro.get_angle())
      steer_motor.track_target(correction)

      c = color.get_color_corrected()
      detected_col = get_color(c)
      if previous_obstacle == 1 and detected_col == ORANGE:
        print("CHEESE")
        lt = RIGHT
        break
      # detected_col = color.get_detected()
      # if detected_col == ORANGE:
      #   gpid.target = prevtarg + RIGHT
      #   print("ORANGEEGEEEE")
      #   break

    # c = color.get_color_corrected()
    # detected_col = get_color(c)
    # if previous_obstacle == 1 and detected_col == ORANGE:
    #   print("CHEESE")
    #   lt = RIGHT
    #   break

      
    
    # drive_motor.brake()
    # break
      


