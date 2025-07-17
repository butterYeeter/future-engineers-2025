import ustruct

class ColorSensor():
  def __init__(self, usb, white=None):
    self.usb = usb
    self.white = white or (0.3, 0.3, 0.3)

  def get_color(self):
    self.usb.write_bytes(b'c', 1)

    buffer = bytearray(12)
    self.usb.read_bytes(buffer, len(buffer))
    color = ustruct.unpack('fff', buffer)

    return color
  
  def get_color_corrected(self):
    raw = self.get_color()
    col= [raw[i]/self.white[i] for i in range(3)]
    wow = max(col[0], max(col[1], col[2]))
    norm = [c/wow for c in col]

    return norm