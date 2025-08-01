import ustruct

class ColorSensor():
  def __init__(self, usb, white=None):
    self.usb = usb
    self.white = white or (0.28571430, 0.28571430, 0.42857143)

  def get_color(self):
    self.usb.write_bytes(b'c', 1)

    buffer = bytearray(12)
    self.usb.read_bytes(buffer, len(buffer))
    color = ustruct.unpack('fff', buffer)

    return color
  
  def get_color_corrected(self):
    raw = self.get_color()
    col = [raw[i]/self.white[i] for i in range(3)]
    max_component = max(col[0], max(col[1], col[2]))
    norm = [c/max_component for c in col]

    return norm