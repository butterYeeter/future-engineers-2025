import ustruct

class Gyro():
  def __init__(self, usb):
    self.usb = usb

  def get_angle(self):
    cmd = b'a'
    self.usb.write_bytes(cmd, len(cmd))

    buffer = bytearray(32)
    self.usb.read_bytes(buffer, len(buffer))

    return ustruct.unpack('f', buffer)[0]
  
  def reset_angle(self):
    cmd = b'r'

    self.usb.write_bytes(cmd, len(cmd))
  
  def calibrate(self, num_samples):
    cmd = bytearray(2)
    cmd.append(b'g')
    cmd.append(num_samples)

    self.usb.write_bytes(cmd, len(cmd))



  def deinit(self):
    self.usb.deinit_usb()