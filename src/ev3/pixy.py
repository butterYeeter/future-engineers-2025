from pybricks.iodevices import I2CDevice

class Pixy(I2CDevice):
  def __init__(self, port, address):
    super().__init__(port, address)
    print("Setting up pixy:")

    getver = bytes((174, 193, 14, 0))
    getres = bytes((174, 193, 12, 1, 0))
    setlamp = bytes((174, 193, 22, 2, 0, 0))

    self.write(0, getver)
    response = self.read(0, 20)
    build = int.from_bytes(response[10:12], 'little')
    print("Firmware version: {}.{}.{}".format(response[8], response[9], build))

    self.write(0, getres)
    response = self.read(0, 10)
    w = int.from_bytes(response[6:8], 'little')
    h = int.from_bytes(response[8:], 'little')
    print("Resolution: {}x{}".format(w, h))

    self.write(0, setlamp)
    response = self.read(0, 10)

  def get_largest_signiture(self):
    cmd = bytes((174, 193, 32, 2, 255, 1))
    self.write(0, cmd)

    response = self.read(0, 20)

    return {
      "type":int.from_bytes(response[6:8], 'little'),
      "cx":int.from_bytes(response[8:10], 'little'),
      "cy":int.from_bytes(response[10:12], 'little'),
      "w":int.from_bytes(response[12:14], 'little'),
      "h":int.from_bytes(response[14:16], 'little'),
    }
  
  def get_largest_block(self):
    sig = self.get_largest_signiture()["type"]
    return {1: "red", 2: "green"}.get(sig, None)

  def get_blocks(self, max_num):
    cmd = bytes((174, 193, 32, 2, 255, max_num))
    self.write(0, cmd)
    sigs = []

    header = self.read(0, 6)

    for i in range(max_num):
      block = self.read(0, 14)
      sigs.append(int.from_bytes(block[0:2], 'little'))

    return (sigs)

  
  def set_lamp(self, on: bool = True):
    cmd = bytes((174,193,22,2,on,0))
    self.write(0, cmd)

    self.read(0, 10)

# p = Pixy()