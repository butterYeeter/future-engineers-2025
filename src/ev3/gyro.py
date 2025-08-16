import ustruct
from pybricks.iodevices import UARTDevice
from pybricks.parameters import Port
u = UARTDevice(Port.S1, 115200)

class Gyro():
  def __init__(self, uart: UARTDevice):
    self.uart = uart

  def get_angle(self):
    cmd = b'a'
    self.uart.write(cmd)

    buffer = self.uart.read(length=4)

    return ustruct.unpack('f', buffer)[0]
  
  def reset_angle(self):
    cmd = b'r'

    self.uart.write(cmd)
  
  def calibrate(self, num_samples):
    cmd = bytearray(2)
    cmd.append(b'g')
    cmd.append(num_samples)

    self.uart.write(cmd)
