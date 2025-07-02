from pybricks.iodevices import I2CDevice

class Pixy(I2CDevice):
  def __init__(self, port, address):
    super().__init__(port, address)

  def get_blocks():
    pass