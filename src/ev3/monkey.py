#!/usr/bin/env pybricks-micropython

from pybricks.ev3devices import Motor
from pybricks.iodevices import UARTDevice, Ev3devSensor
from pybricks.parameters import Port
from pybricks.tools import wait
from util import rgb_to_hsv, is_blue

# m = Motor(Port.B)

# m.dc(100)

# u = UARTDevice(Port.S1, 115200, 10)\
u = Ev3devSensor(Port.S1)
c = Ev3devSensor(Port.S2)

while True:
  # print(u.read('US-DIST-CM'))
  col = c.read('NORM')[:3]
  if is_blue(col):
    wait(5000)

  print(col)