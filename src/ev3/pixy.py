from pybricks.iodevices import I2CDevice

class Pixy(I2CDevice):
  def __init__(self, port, address):
    super().__init__(port, address)
    print("Setting up pixy:")

    getver = bytes((174, 193, 14, 0))

    self.write(None, getver)

    response = self.read(None, 12)

    print(f"Firmware version: {int(response[8])}.{int(response[9])}")

  def get_blocks():
    pass