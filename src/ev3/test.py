#!/usr/bin/env pybricks-micropython
from usb import USB
from pixy import Pixy
from color import ColorSensor
from gyro import Gyro
from pid import PIDController
from pybricks.ev3devices import Motor, UltrasonicSensor
from pybricks.iodevices import UARTDevice
from pybricks.parameters import Port, Stop
from util import *
from pybricks.tools import DataLog


LEFT = 90
RIGHT = -90
SPEED = 800
DISTANCE = 200

steer_motor = Motor(Port.D)
drive_motor = Motor(Port.B)

pid = PIDController(target=316//2, Kp=0.25, Kd=0.05, output_limit=40)
pixy = Pixy(Port.S2, 0x54)

pico_uart = UARTDevice(Port.S4, 115200)
pico_uart.clear()
color = ColorSensor(pico_uart, (0.43243244, 0.32432431, 0.21621622))
gyro = Gyro(pico_uart)
left_us = UltrasonicSensor(Port.S1)
right_us = UltrasonicSensor(Port.S3)
target_angle = gyro.get_angle()
target_distance = DISTANCE
gpid = PIDController(target_angle, 2.2, 0.02, 1.5, 32, 5)
upid = PIDController(target_distance, 0.8, 0.01, 0.7, 35, 100)
directions = {
  "l": 1,
  "r": -1
}

calib_steering(steer_motor)


log = DataLog("", name="../log", timestamp=False)

# drive_motor.run(SPEED)
# dodging = False
# dodge_obstacle = 0
# change = 30
# counter = 0
# precision = 0
# previous_obstacle = 0
# lt = False

def clamp(val, min_val, max_val):
  return max(min_val, min(val, max_val))

def sign(val):
  return val/abs(val)

def turn_to_angle(angle=90, direction="l", power=50, steering=40, compensation=10, precision=1):
  """Turns the robot to a specific target angle

  int power - Value from -100 to 100 where positive is forwards

  int steering - Steering motor angle from 1 to 42

  int angle - Angle to which to turn in degrees(>0)
  
  int precision - How close to the target angle the robot needs to be in degrees before ending the turn

  int compensation - How much to compensate in angle to prevent overshooting the turning angle

  str direction - Opiton for turning left or right ("l" or "r", not sensitive, e.g Right, Left, L, R, l, r, Ricardo, Leonardo, etc.)
  """
  print("Starting Angle: {}".format(gyro.get_angle()))
  direction = directions[direction[0].lower()]
  steer_motor.run_angle(1000, 0, then=Stop.HOLD,wait=True)
  angle = gyro.get_angle()+((abs(angle)-compensation)*sign(power)*direction)
  print("Target Angle: {}".format(angle))
  steering = clamp(abs(steering), 1, 40)*direction
  print("Steering Angle: {}".format(steering))
  # steer_motor.run_angle(1000, steering, then=Stop.HOLD,wait=True)
  steer_motor.track_target(steering)
  curr = gyro.get_angle()

  drive_motor.dc(power)
  while abs(curr-angle)>=precision:
    print("Current Gyro Angle: {}".format(curr))
    print("Difference: {}".format(abs(curr-angle)))
    curr = gyro.get_angle()
  drive_motor.hold()


def set_steer(target_deg):
  steer_motor.track_target(target_deg)

#turn_to_angle(30, "left")
#turn_to_angle(30, "right", -50)
def parallel_park(straight_angle):
  wall_counter = 0
  upid.target = 270
  drive_motor.dc(60)
  last_dist = right_us.distance()
  last_angle = gyro.get_angle()
  gyro_weight = 0.5
  us_weight = 0.5
  while wall_counter < 2:
    dist = right_us.distance()
    delta_dist = abs(dist - last_dist)
    last_dist = dist

    angle = gyro.get_angle()
    delta_angle = abs(angle - straight_angle)
    last_angle = angle

    # us_weight = 0.6
    # gyro_weight = 0.4
    if delta_angle > 37:
      us_weight = 0
      gyro_weight = 1
    else:
      us_weight = 0.5
      gyro_weight = 0.5

    if delta_dist > 150 and abs(dist - upid.target) < 50:
      us_weight *= 0.5
      wall_counter += 1
      print("Wall")

    correction = us_weight * upid.loop(dist) + gyro_weight * gpid.loop(angle)
    set_steer(correction)
  drive_motor.hold()


  steer_motor.run_target(1000, 0)
  drive_motor.run_angle(1000, distance_to_angle(50))
  # motor_angle = drive_motor.angle() + distance_to_angle(50)
  # while drive_motor.angle() < motor_angle:
    # angle = gyro.get_angle()
    # correction = gpid.loop(angle)
    # steer_motor.run_target(1000, correction, wait=False)

  turn_to_angle(60, "Right", -80)

  turn_to_angle(45, "left", -80, compensation=3)
  turn_to_angle(15,"right", compensation=3)
  steer_motor.run_target(1000, 0)

# turn_to_angle(15,"rift", -50, compensation=3)
# turn_to_angle(45, "light", 80)
# turn_to_angle(60, "reft", 80)
