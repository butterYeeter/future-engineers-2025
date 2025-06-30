#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.tools import wait
import ffi
import sys

ev = EV3Brick()

usbser = ffi.open('./libusbser.so')

init = usbser.func('i', 'init_usb', 'si')
inflush = usbser.func('v', 'flush_input', '')
angle = usbser.func('f', 'angle', '')
deinit = usbser.func('v', 'deinit', '')

port = '/dev/ttyACM0'
baud = 115200

if init(port, baud) == 0:
    ev.screen.print("SUCCESS")
    inflush()
    wait(1000)
else:
    usbser.close()
    ev.screen.print("FAILED")
    wait(2000)
    sys.exit(1)


for i in range(100):
    print(angle())
    wait(100)



wait(5000)
deinit()
usbser.close()
