from gyro import Gyro
import utime

g = Gyro()

g.init()

while True:
  print(g.get_angle())
  utime.sleep_ms(200)

