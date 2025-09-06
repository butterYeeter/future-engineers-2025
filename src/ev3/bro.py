#!/usr/bin/env pybricks-micropython
from pybricks.ev3devices import Motor, UltrasonicSensor
from pybricks.iodevices import UARTDevice
from pybricks.parameters import Port
from pybricks.tools import StopWatch

from util import *
from color import ColorSensor
from gyro import Gyro
from pixy import Pixy
from pid import PIDController
from math import sin, radians

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
PIXY_PID_PARAMS = dict(Kp=0.25, Kd=0.05, output_limit=42)
WALL_PID_PARAMS = dict(Kp=1.0, output_limit=20)

# Constants
BLUE = 1                    # This represents the blue color
ORANGE = 2                  #          ''         orange color
LEFT = 90                   # Counter-clockwise turn 90 degrees
RIGHT = -90                 # Clockwise turn 90 degrees
PIXY_MIN_X = 50             # Minimum x coordinate for us to consider tracking an object
PIXY_MAX_X = 266            # Maximum x coordinate                 ''
BLOCK_AREA_THRESH = 3000    # Area threshold before we dodge an obstacle
RECENTER_THRESH = 50        # Error threshold for how far off our recentering can be (units in mm)
DISTANCE_MIN = 50           # Minimum valid distance (mm)
DISTANCE_MAX = 1000         # Maximum valid distance (mm)
RECENTER_ANGLE = 45         # Angle to turn when recentering (degrees)

# Initialize sensors
uart = UARTDevice(PICO_PORT, PICO_BAUD, 50)       # UART device for pico

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
timer = StopWatch()


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
  
def valid_distance(dist):
  return DISTANCE_MIN < dist and dist < DISTANCE_MAX

def l_turn(dir):
  dist = drive_motor.angle() + distance_to_angle(500)

  angle = gyro.get_angle()
  while drive_motor.angle() < dist:
    angle = gyro.get_angle()
    steer_cmd = gyro_pid.loop(angle)
    set_steer(steer_cmd)

  gyro_pid.target += dir * 90
  set_drive(-70)
  while abs(angle - gyro_pid.target) > 3.0:
    angle = gyro.get_angle()
    steer_cmd = -gyro_pid.loop(angle)
    set_steer(steer_cmd)

  # set_drive(-60)
  print("TIMER")
  timer = StopWatch()
  start = timer.time()
  drive_motor.run(-1000)
  while timer.time() - start < 2000:
    angle = gyro.get_angle()
    steer_cmd = -gyro_pid.loop(angle)
    set_steer(steer_cmd)
  set_drive(70)
  


# Current state
state="CRUISE"
prev_heading = gyro_pid.target
prev_angle = gyro.get_angle()
motor_angle_target = 0
recenter_dir = 1
last_obstacle = 0
last_turn_time = timer.time() - 3000
set_drive(70)

while True:
  angle = gyro.get_angle()
  detected_color = color.get_detected()
  turn = should_turn(detected_color)
  dt = timer.time() - last_turn_time
  if dt < 3000:
    turn = False
    color.reset_detected()
  block = pixy.get_largest_block()
  left = left_us.distance()
  right = right_us.distance()

  # if  block != None:
  #   if block["cx"] > PIXY_MAX_X or block["cx"] < PIXY_MIN_X:
  #     block = None

  # print("STATE: {} COLOR: {} SHOULD_TURN: {} DT: {}".format(state, detected_color, turn, dt))

  # red_lturn = detected_color == ORANGE and last_obstacle == 1
  # green_lturn = detected_color == BLUE and last_obstacle == 2
  # if red_lturn or green_lturn:
  #   state = "LTURN"

  if state == "CRUISE":
    steer_cmd = gyro_pid.loop(angle)
    set_steer(steer_cmd)

    if turn:
      prev_heading = gyro_pid.target
      state = "CORNER_TURN"
      continue

    if block != None:
      state = "TRACK_OBJECT"

  elif state == "TRACK_OBJECT":
    # state = "CRUISE"
    if block == None:
      state = "CRUISE"
      continue

    steer_cmd = pixy_pid.loop(316-block["cx"])
    set_steer(steer_cmd)

    if block["w"] * block["h"] > BLOCK_AREA_THRESH:
      prev_heading = gyro_pid.target
      prev_angle = angle
      state = "OVERTAKE-RIGHT" if block["type"] == 1 else "OVERTAKE-LEFT"
      motor_angle_target = drive_motor.angle() + distance_to_angle(200)
      last_obstacle = block["type"]


  elif state.startswith("OVERTAKE"):
    dir = 1 if state.endswith("LEFT") else -1
    gyro_pid.target = prev_heading + dir * 45
    steer_cmd = gyro_pid.loop(angle)
    set_steer(steer_cmd)
    
    if drive_motor.angle() > motor_angle_target:
      state = "CRUISE"
      gyro_pid.target = prev_heading



    # recenter_dir = 1 if left - 400 > 0 else -1
    # if not valid_distance(left):
    #   recenter_dir = -1 if right - 400 > 0 else 1
    # dist = left if valid_distance(left) else right
    # base = drive_motor.angle()
    # motor_angle_target = base + distance_to_angle(abs(dist - 385) / sin(radians(RECENTER_ANGLE)))
    # prev_heading = gyro_pid.target
    # gyro_pid.target += recenter_dir * RECENTER_ANGLE
    # state = "RECENTER"
    # print(dist)
    # print(motor_angle_target)
    
  elif state == "LTURN":
    dir = 1 if detected_color == BLUE else -1
    l_turn(dir)
    state = "CRUISE"
    last_obstacle = 0
    color.reset_detected()
    last_turn_time = timer.time()

  elif state == "RECENTER":
    # gyro_pid.output_limit = 12
    # base = gyro_pid.loop(angle)

    # lr_error = left_us.distance() - right_us.distance()
    # bias = wall_pid.loop(lr_error)

    # set_steer(base + bias)

    # if abs(lr_error) < 30:
    #   gyro_pid.output_limit = 42
    #   state = "CRUISE"
    #   continue

    steer_cmd = gyro_pid.loop(angle)
    set_steer(steer_cmd)

    if drive_motor.angle() > motor_angle_target:
      gyro_pid.target = prev_heading
      state = "CRUISE"
      continue

    
    

      
  elif state == "CORNER_TURN":
    turn_dir = LEFT if detected_color == BLUE else RIGHT

    gyro_pid.target = prev_heading + turn_dir
    
    steer_cmd = gyro_pid.loop(angle)
    set_steer(steer_cmd)

    if abs(angle - gyro_pid.target) < 3.0:
      set_steer(0)
      # color.reset_detected()
      last_turn_time = timer.time()
      state = "CRUISE"