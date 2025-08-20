#!/usr/bin/env pybricks-micropython
from gyro import Gyro
from color import ColorSensor
from pybricks.iodevices import Ev3devSensor, UARTDevice
from pixy import Pixy
from pybricks.ev3devices import Motor
from pybricks.hubs import EV3Brick
from pybricks.parameters import Port, Direction
from pybricks.tools import DataLog
from pid import PIDController
from calib import calib_steering
from util import *

# PARAMETERS
LEFT_US_PORT = Port.S1
PIXY_PORT = Port.S2
PICO_PORT = Port.S4
PICO_BAUD = 115200
RIGHT_US_PORT = Port.S4
NUM_ROUNDS = 3
WHITE_REFERENCE = (0.4423076808452606, 0.3269230723381042, 0.2307692319154739)
LEFT = 90
RIGHT = -90

# Robot class that has driving logic and sensors
class Robot:
    def __init__(self):
        # Open up the UART port for the pico to ev3 communication
        pico_uart = UARTDevice(PICO_PORT, PICO_BAUD)
        pico_uart.clear()

        # Init gyro and color sensor classes using previously opened UART port
        self.gyro = Gyro(pico_uart)
        self.color = ColorSensor(pico_uart, WHITE_REFERENCE)

        # Setup up steering and driving motor
        self.steer_motor = Motor(Port.D)
        self.drive_motor = Motor(Port.B, positive_direction=Direction.CLOCKWISE)
        calib_steering(self.steer_motor)

        # Setup ultrasonic and ld_prev variable
        self.ultrasonic = Ev3devSensor(Port.S1)
        self.ld_prev = self.ultrasonic.read('US-DIST-CM')[0] # stores the previous iterations left distance

        # Set the target angle to the current heading and target distance to current distance
        self.target_angle = self.gyro.get_angle()
        self.target_distance = self.ultrasonic.read('US-DIST-CM')[0]

        # Setup the ultrasonic and gyro PID controllers
        self.pid1 = PIDController(self.target_distance, 0.8, 0.01, 0.7, 35, 100)
        self.pid2 = PIDController(self.target_angle, 2.2, 0.02, 1.5, 42, 5)

        # Setup pixy cam on 
        self.pixy = Pixy(PIXY_PORT, 0x54)

        # Additional variables
        self.turning = False
        self.ultrasonic_weight = 0.5
        self.gyro_weight = 0.5 
        self.num_turns = 0
        self.parking_mode = False
        self.parking_wall_counts = 0

        self.log = DataLog("RGB", "HSV", "../log", timestamp=False)

    # Function to get teh current ultrasonic reading and the delta angle
    def get_new_readings(self):
        left_distance = self.ultrasonic.read('US-DIST-CM')[0]
        delta_angle = abs(self.target_angle - self.gyro.get_angle())
        return left_distance, delta_angle 

    # Function to begin a turning operation
    def start_turn(self, angle=0):
        if self.turning:
            return
        self.turning = True
        self.target_angle += angle
        self.pid2.set_target(self.target_angle)
        self.num_turns += 1

    # Function that handles turning. Sets the ultrasonic controllers wait to zero.
    # Also sets up a cooldown.
    def handle_turning(self, left, delta_angle):
        if self.turning:
            self.ultrasonic_weight = 0
            
            # If we are within 5 degrees of our target angle, we can restore the ultrasonic controller's weight variable
            if delta_angle < 5:
                self.turning = False
                self.ultrasonic_weight = 0.5
        else:
            # Reduce the weight of the ultrasonic distance sensor incase it might be misreporting values
            if abs(left - self.ld_prev) > 500:
                self.ultrasonic_weight *= 0.8

    # Function that contains the main loop and logic
    def drive(self):
        # Start the drive motor
        self.drive_motor.run(800)
        
        # Main loop
        while True:
            # Get current readings for delta_angle and the left distance
            left_distance, delta_angle = self.get_new_readings()

            # If the delta_angle is greater than 30 degrees we should set the ultrasonic weight to zero
            # because the ultrasonic reports very wrong distances
            if delta_angle > 30:
                self.ultrasonic_weight = 0
            else:
                self.ultrasonic_weight = 0.5

            # Call handle turning function
            self.handle_turning(left_distance, delta_angle)

            # Get the corrected color value(rgb) from the sensor
            color = self.color.get_color_corrected()
            detected_color = get_color(color)   # What color do we see
            
            # Turn in the appropriate direction based on the detected color
            if detected_color == BLUE:
                print("Turning left...")
                self.start_turn(LEFT)
            elif detected_color == ORANGE:
                print("Turning RIGHT")
                self.start_turn(RIGHT)

            # Break out of the loop once we've completed the final turn
            if self.num_turns == NUM_ROUNDS * 4 and self.turning == False:
                break

            # Set ld_prev to the current left_distance
            self.ld_prev = left_distance

            # Calculate the current steering correction based on our PID controllers
            correction = self.gyro_weight * self.pid2.loop(self.gyro.get_angle()) + self.ultrasonic_weight * -self.pid1.loop(left_distance)
            self.steer_motor.track_target(correction)   # Use correction to steer

        # Reset the drive motors encode to 0 degrees
        self.drive_motor.reset_angle(0)
        self.drive_motor.run(800)
        fifty_cm = distance_to_angle(500) # 50 cm as degrees

        # Drive for fifty centimeters
        while self.drive_motor.angle() < fifty_cm:
            # Use our gyro to drive straight
            correction = self.gyro_weight * self.pid2.loop(self.gyro.get_angle())
            self.steer_motor.track_target(correction)

        # Brake at the end :)
        self.drive_motor.brake()