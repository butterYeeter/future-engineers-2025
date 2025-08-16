import ustruct
from pybricks.iodevices import UARTDevice

COLOR_VECTOR_LENGTH=12

class ColorSensor():
  def __init__(self, uart: UARTDevice, white=None):
    self.uart = uart
    self.white = white or (0.28571430, 0.28571430, 0.42857143)

  def get_color(self):
    self.uart.write(b'c')

    buffer = self.uart.read(length=COLOR_VECTOR_LENGTH)
    color = ustruct.unpack('fff', buffer)

    return color
  
  def get_color_corrected(self):
    raw = self.get_color()
    col = [raw[i]/self.white[i] for i in range(3)]
    max_component = max(col[0], max(col[1], col[2]))
    norm = [c/max_component for c in col]

    return norm