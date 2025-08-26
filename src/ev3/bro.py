#!/usr/bin/env pybricks-micropython
from pybricks.ev3devices import Motor, UltrasonicSensor
from pybricks.iodevices import UARTDevice
from pybricks.parameters import Port

from util import *
from color import ColorSensor
from gyro import Gyro
from pixy import Pixy
from pid import PIDController


# Sensors config
# Ports for our motors and sensors. Also contains baudrate for pico and i2c address for pixy
STEER_M_PORT = Port.D
DRIVE_M_PORT = Port.B

LEFT_US_PORT = Port.S1
RIGHT_US_PORT = Port.S3
PIXY_PORT = Port.S2
PIXY_ADDR = 0x54
PICO_PORT = Port.S4
PICO_BAUD = 115200

# PID controllers config
# Gyro PID is for driving forwards and pixy PID is for tracking objects
GYRO_PID_PARAMS = dict(Kp=2.2, Ki=0.02, Kd=0.3, output_limit=42, integral_limit=5)
PIXY_PID_PARAMS = dict(Kp=0.25, Kd=0.05, output_limit=40)
WALL_PID_PARAMS = dict(Kp=1.0, output_limit=20)

# Constants
BLUE = 1                    # This represents the blue color
ORANGE = 2                  #          ''         orange color
LEFT = 90                   # Counter-clockwise turn 90 degrees
RIGHT = -90                 # Clockwise turn 90 degrees
PIXY_MAX_X = 256            # Maximum x coordinate for us to consider tracking an object
PIXY_MIN_X = 60             # Minimum x coordinate                 ''
BLOCK_AREA_THRESH = 3000    # Area threshold before we dodge an obstacle
RECENTER_THRESH = 50        # Units in mm

# Initialize sensors
uart = UARTDevice(PICO_PORT, PICO_BAUD)       # UART device for pico

left_us = UltrasonicSensor(LEFT_US_PORT)      # Left ultrasonic
right_us = UltrasonicSensor(RIGHT_US_PORT)    # Right   ''
pixy = Pixy(PIXY_PORT, PIXY_ADDR)             # Pixy cam 
color = ColorSensor(uart)                     # Color sensor from pico
color.reset_detected()
gyro = Gyro(uart)                             # Gyro sensor from pico

# Initialize PID controllers
gyro_pid = PIDController(gyro.get_angle(), **GYRO_PID_PARAMS)
pixy_pid = PIDController(316//2, **PIXY_PID_PARAMS)
wall_pid = PIDController(0, **WALL_PID_PARAMS)

# Initialize motors
steer_motor = Motor(STEER_M_PORT)     # Steering motor
calib_steering(steer_motor)           # Recenter the motor's 0 angle
drive_motor = Motor(DRIVE_M_PORT)     # Drive motor

# Current state
state="RECENTER"

def clamp(val, min_val, max_val):
  return max(min_val, min(val, max_val))

def set_drive(dc):
  drive_motor.dc(clamp(dc, -100, 100))

def set_steer(target_deg):
  steer_motor.track_target(target_deg)

def should_turn(color):
  if color in (BLUE, ORANGE):
    return True
  else:
    return False


set_drive(70)
pre_turn_dir = gyro_pid.target

while True:
  angle = gyro.get_angle()
  detected_color = color.get_detected()
  turn = should_turn(detected_color)
  block = pixy.get_largest_signiture()
  if block["cx"] > PIXY_MAX_X or block["cx"] < PIXY_MIN_X:
    block = None

  if state == "CRUISE":
    steer_cmd = gyro_pid.loop(angle)
    set_steer(steer_cmd)

    if turn:
      pre_turn_dir = gyro_pid.target
      state = "CORNER_TURN"
      continue

    if block != None:
      state = "TRACK_OBJECT"

  elif state == "TRACK_OBJECT":
    # state = "CRUISE"
    steer_cmd = pixy_pid.loop(316-block["cx"])
    set_steer(steer_cmd)

    if block["w"] * block["h"] > BLOCK_AREA_THRESH:
      state = "OVERTAKE"


  elif state.startswith("OVERTAKE"):
    print("left dist: {}, right dist: {}".format(left_us.distance(), right_us.distance()))
    drive_motor.brake()

  elif state == "RECENTER":
    gyro_pid.output_limit = 12
    base = gyro_pid.loop(angle)

    lr_error = left_us.distance() - right_us.distance()
    bias = wall_pid.loop(lr_error)

    set_steer(base + bias)

    if abs(lr_error) < 30:
      gyro_pid.output_limit = 42
      state = "CRUISE"
      continue

      
  elif state == "CORNER_TURN":
    turn_dir = LEFT if detected_color == BLUE else RIGHT

    gyro_pid.target = pre_turn_dir + turn_dir
    
    steer_cmd = gyro_pid.loop(angle)
    set_steer(steer_cmd)

    if abs(angle - gyro_pid.target) < 3.0:
      set_steer(0)
      color.reset_detected()
      state = "CRUISE"