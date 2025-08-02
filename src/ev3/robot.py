#!/usr/bin/env pybricks-micropython
from usb import USB
from gyro import Gyro
from color import ColorSensor
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import UltrasonicSensor, Motor
from pybricks.iodevices import Ev3devSensor
from pybricks.parameters import Port, Direction, Button
from pybricks.tools import DataLog, StopWatch, wait
import utime
from math import cos, radians
from pid import PIDController
from sys import argv
from calib import calib_steering
from util import is_blue, is_orange
from pixy import Pixy

class Robot:
    def __init__(self):
        usb = USB()

        # (0.43, 0.34, 0.21)
        self.gyro = Gyro(usb)
        self.color = ColorSensor(usb, (0.44897959, 0.32653061, 0.20408164))

        self.steer_motor = Motor(Port.C)
        self.drive_motor = Motor(Port.B, positive_direction=Direction.CLOCKWISE)
        calib_steering(self.steer_motor)

        self.ultrasonic = Ev3devSensor(Port.S1)
        self.ld_prev = self.ultrasonic.read('US-DIST-CM')[0]

        self.target_angle = self.gyro.get_angle()
        self.target_distance = self.ultrasonic.read('US-DIST-CM')[0]

        self.pid1 = PIDController(self.target_distance, 0.8, 0.01, 0.7, 35, 100)
        self.pid2 = PIDController(self.target_angle, 2.2, 0.02, 1.5, 42, 5)

        self.turning = False
        self.cooldown = 0

        self.ultrasonic_weight = 0.5
        self.gyro_weight = 0.5 

        self.pixy = Pixy(Port.S2, 0x54)
        self.num_turns = 0

        self.log = DataLog("RGB", "HSV", "../log", timestamp=False)
        self.finish_cooldown = 0

    def get_new_readings(self):
        left_distance = self.ultrasonic.read('US-DIST-CM')[0]
        delta_angle = abs(self.target_angle - self.gyro.get_angle())
        return left_distance, delta_angle 

    def start_turn(self, angle=90):
        if self.turning:
            return
        self.turning = True
        self.cooldown = 150
        self.target_angle += angle
        self.pid2.set_target(self.target_angle)
        self.num_turns += 1

    def handle_turning(self, left):
        if self.turning:
            self.cooldown -= 1
            self.ultrasonic_weight = 0
            if not self.cooldown:
                self.turning = False
                self.ultrasonic_weight = 0.5
        else:
            if left - self.ld_prev > 500:
                self.ultrasonic_weight *= 0.8

    def drive(self):
        self.drive_motor.dc(90)
        # while True:
        #     c = self.color.get_color()
        #     print(c)

        while self.num_turns < 13:
            left_distance, delta_angle = self.get_new_readings()

            if delta_angle > 30:
                self.ultrasonic_weight = 0
            else:
                self.ultrasonic_weight = 0.5

            self.handle_turning(left_distance)

            # block_color = self.pixy.get_largest_block()
            # if block_color == "green":
            #     print("Found a green block! moving into left lane")
            #     self.target_distance = 150
            #     self.pid1.set_target(self.target_distance)
            color = self.color.get_color_corrected()
            do_turn = is_blue(color, self.log)
            if do_turn:
                print("blue turn")
                self.start_turn(angle=90)
            do_turn = is_orange(color, self.log)
            if do_turn:
                print("orange turn")
                self.start_turn(angle=-90)

            if self.num_turns == 11:
                self.finish_cooldown = 150

            if self.finish_cooldown > 0:
                if self.finish_cooldown == 1:
                    break 
                self.finish_cooldown -= 1

            self.ld_prev = left_distance
            # self.ultrasonic_weight = 0
            correction = self.gyro_weight * self.pid2.loop(self.gyro.get_angle()) + self.ultrasonic_weight * -self.pid1.loop(left_distance)
            self.steer_motor.track_target(correction)

# JetBrainsMono NFM SemiBold, Consolas, 'Courier New', monospace