import ustruct
from pybricks.iodevices import UARTDevice

# Gyro sensor class
class Gyro():
  def __init__(self, uart: UARTDevice):
    self.uart = uart

  # Function to get current accumulated angle
  def get_angle(self):
    cmd = b'a'
    self.uart.write(cmd)

    buffer = self.uart.read(length=4)

    return ustruct.unpack('f', buffer)[0]
  
  # Function to reset the gyro's accumulated angle
  def reset_angle(self):
    cmd = b'r'

    self.uart.write(cmd)
  
  # Calibrate the gyro using 100 * num_samples values
  def calibrate(self, num_samples):
    cmd = bytearray(2)
    cmd.append(b'g')
    cmd.append(num_samples)

    self.uart.write(cmd)
