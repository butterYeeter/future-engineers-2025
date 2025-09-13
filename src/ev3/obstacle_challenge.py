#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, UltrasonicSensor
from pybricks.iodevices import UARTDevice
from pybricks.parameters import Port, Color
from pybricks.tools import StopWatch

from util import *
from color import ColorSensor
from gyro import Gyro
from pixy import Pixy
from pid import PIDController
from math import sin, radians
from park import parallel_park

brick = EV3Brick()
brick.light.off()
brick.speaker.set_speech_options(language="en-gb", voice="m2", speed=150,pitch=80)
brick.speaker.set_volume(60, "PCM")
brick.speaker.say("I am Huebert.")
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
PIXY_PID_PARAMS = dict(Kp=0.25, Kd=0.05, output_limit=38, integral_limit=10)
WALL_PID_PARAMS = dict(Kp=1.5, Ki=0.02, Kd=1.2, output_limit=40)

# Constants
BLUE = 1                    # This represents the blue color
ORANGE = 2                  # This represents the orange color
LEFT = 90                   # Counter-clockwise turn 90 degrees
RIGHT = -90                 # Clockwise turn 90 degrees
PIXY_MIN_X = 22             # Minimum x coordinate for us to consider tracking an object
PIXY_MAX_X = 293            # Maximum x coordinate for us to consider tracking an object
MAX_BLOCK_AREA = 3400        # Area threshold before we dodge an obstacle
MIN_BLOCK_AREA = 650        # Minimum area threshold before we track an obstacle
RECENTER_THRESH = 50        # Error threshold for how far off our recentering can be (units in mm)
DISTANCE_MIN = 50           # Minimum valid distance (mm)
DISTANCE_MAX = 1000         # Maximum valid distance (mm)
RECENTER_ANGLE = 45         # Angle to turn when recentering (degrees)
GYRO_WEIGHT = 0.8           # Weight of the gyro PID controller when driving straight
US_WEIGHT = 0.2             # Weight of the ultrasonic PID controller when driving straight
RED_SIG = 1                 # Signature index for the red object
GREEN_SIG = 4               # Signature index for the green object
NUM_LAPS = 3                # Configure number of laps to do


# Initialize sensors
uart = UARTDevice(PICO_PORT, PICO_BAUD, 50)       # UART device for pico

left_us = UltrasonicSensor(LEFT_US_PORT)      # Left ultrasonic
right_us = UltrasonicSensor(RIGHT_US_PORT)    # Right   ''
pixy = Pixy(PIXY_PORT, PIXY_ADDR)             # Pixy cam 
color = ColorSensor(uart, (0.4375, 0.3125, 0.21875))                     # Color sensor from pico
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
  timer = StopWatch()
  dist = drive_motor.angle() + distance_to_angle(550)

  set_drive(75)
  # angle = gyro.get_angle()
  # while drive_motor.angle() < dist:
  start = timer.time()
  while timer.time() - start < 3500:
    angle = gyro.get_angle()
    steer_cmd = gyro_pid.loop(angle)
    set_steer(steer_cmd)

  gyro_pid.target += dir * 90
  set_drive(-75)
  while abs(angle - gyro_pid.target) > 2.0:
    angle = gyro.get_angle()
    steer_cmd = -gyro_pid.loop(angle)
    set_steer(steer_cmd)

  # set_drive(-60)
  print("TIMER")
  start = timer.time()
  drive_motor.run(-1000)
  while timer.time() - start < 2000:
    angle = gyro.get_angle()
    steer_cmd = -gyro_pid.loop(angle)
    set_steer(steer_cmd)
  set_drive(75)
  
brick.speaker.beep(duration=500)
wait_for_start()


# Current state
state="CRUISE"
start_of_round_heading = gyro.get_angle()
challenge_direction = 1 # CCW
end_of_round_heading = challenge_direction * 360 * NUM_LAPS + start_of_round_heading
prev_heading = gyro_pid.target
gyro_weight = GYRO_WEIGHT
us_weight = US_WEIGHT
angle = gyro.get_angle()
prev_angle = angle
motor_angle_target = 0
recenter_dir = 1
last_obstacle = 0
last_turn_time = timer.time() - 1500
turns = 0
set_drive(75)
while not (turns >= NUM_LAPS*4 and abs(angle-end_of_round_heading) < 5):
  end_of_round_heading = challenge_direction * 360 * NUM_LAPS + start_of_round_heading


  angle = gyro.get_angle()
  delta_angle = angle - gyro_pid.target
  prev_angle = angle

  print("end condition: {}".format(abs(angle-(challenge_direction * 90 * 4 + start_of_round_heading))))

  right_dist = right_us.distance()
  left_dist = left_us.distance()
  lane_width = right_dist + left_dist
  delta_dist =  right_dist - left_dist
  abs_delta_dist = abs(delta_dist)

  time_since_last_turn = timer.time() - last_turn_time
  detected_color = color.get_detected()
  if time_since_last_turn < 4000:
    detected_color = 0
    color.reset_detected()

  
  block = pixy.get_largest_block()
  if  block != None:
    if block["cx"] > PIXY_MAX_X or block["cx"] < PIXY_MIN_X:
      block = None


  red_lturn = detected_color == ORANGE and last_obstacle == RED_SIG
  green_lturn = detected_color == BLUE and last_obstacle == GREEN_SIG
  if (red_lturn or green_lturn):
    state = "LTURN"
    if turns == 0:
      challenge_direction = 1 if detected_color == BLUE else -1
    turns += 1
  elif detected_color != 0:
    heading_change = LEFT if detected_color == BLUE else RIGHT
    if turns == 0:
      challenge_direction = 1 if detected_color == BLUE else -1
    turns += 1
    if turns == NUM_LAPS * 4:
      state = "LTURN"
    else:
      gyro_pid.target += heading_change
    last_turn_time = timer.time()


  if detected_color == BLUE:
    brick.light.on(Color.YELLOW)
    # brick.speaker.beep(800, 500)
  elif detected_color == ORANGE:
    brick.light.on(Color.ORANGE)

    # brick.speaker.beep(900, 500)
  # else:
  #   color.reset_detected()

  # print("STATE: {} COLOR: {} DT: {} TARGET_ANGLE: {} LAST_OBJECT: {}".format(state, detected_color, dt, gyro_pid.target, last_obstacle))

  if state == "CRUISE":
    print("Lane width: {}\nDelta angle: {}".format(lane_width, delta_angle))
    if abs(delta_angle) > 32 or lane_width < 400 or lane_width > 1200:
      us_weight = 0
      gyro_weight = 1
      #brick.speaker.play_notes(["C4/8", "G4/8", "C5/8"], tempo=360)
    elif abs_delta_dist > 200 and abs_delta_dist < 1000:
      us_weight = 0.6
      gyro_weight = 0.4
    else:
      us_weight = US_WEIGHT
      gyro_weight = GYRO_WEIGHT

    if abs(abs_delta_dist) > 1000:
      us_weight *= 0.6


    steer_cmd = gyro_pid.loop(angle) * gyro_weight + wall_pid.loop(delta_dist) * us_weight
    set_steer(steer_cmd)

    if block != None:
      area = block["w"] * block["h"]
      state = "TRACK_OBJECT" if area > MIN_BLOCK_AREA else "CRUISE"

  if state == "TRACK_OBJECT":
    if block == None:
      state = "CRUISE"
      continue

    area = block["w"] * block["h"]
    pixy_weight = 1 if area > 1700 else 0.42

    steer_cmd = pixy_pid.loop(316-block["cx"]) * pixy_weight
    set_steer(steer_cmd)

    if  area > MAX_BLOCK_AREA:
      if block["type"] == RED_SIG: brick.speaker.beep(duration=200)
      else: brick.speaker.beep(650, 200)
      state = "OVERTAKE-RIGHT" if block["type"] == RED_SIG else "OVERTAKE-LEFT"
      distance = 230 if abs(delta_angle) < 40 else 250
      motor_angle_target = drive_motor.angle() + distance_to_angle(distance)
      last_obstacle = block["type"]
      prev_heading = gyro_pid.target

      if state.endswith("RIGHT"):
        gyro_pid.target = (gyro_pid.target if delta_angle < 0 else angle) -45
      else:
        gyro_pid.target = (gyro_pid.target if delta_angle > 0 else angle) +45


  if state.startswith("OVERTAKE"):
    # print("Motor: {}, target: {}, angle: {}, angle_targ: {}".format(drive_motor.angle(), motor_angle_target, angle, gyro_pid.target))
    steer_cmd = gyro_pid.loop(angle)
    set_steer(steer_cmd)

    if drive_motor.angle() > motor_angle_target and state != "OVERTAKE":
      direction = 1 if state.endswith("RIGHT") else -1
      state = state[0:8]
      print("state: {}".format(state))
      gyro_pid.target += direction * 45

    if lane_width < 600:
      state = "CRUISE"
      gyro_pid.target = prev_heading
    
    
  if state == "LTURN":
    dir = 1 if detected_color == BLUE else -1
    l_turn(dir)
    state = "CRUISE"
    # last_obstacle = 0
    color.reset_detected()
    last_turn_time = timer.time()


print("START ANGLE: {} CURRENT ANLGE: {} END ANGLE: {}".format(start_of_round_heading, gyro.get_angle(), end_of_round_heading))

brick.speaker.beep(2000, 200)
parallel_park(end_of_round_heading)