import ustruct
from pybricks.iodevices import UARTDevice

# PARAMETERS
COLOR_VECTOR_LENGTH=12
DEFAULT_WHITE =  (0.42857143, 0.28571430, 0.28571430)

# Color Sensor Class
class ColorSensor():
  def __init__(self, uart: UARTDevice, white=None):
    self.uart = uart
    self.white = white or DEFAULT_WHITE # Set white reference to default if none is provided

  # Function to get the uncorrected color
  def get_color(self):
    self.uart.write(b'c')

    buffer = self.uart.read(length=COLOR_VECTOR_LENGTH)
    color = ustruct.unpack('fff', buffer)

    return color
  
  # Function that returns the corrected color using the white reference
  @micropython.native
  def get_color_corrected(self):
    raw = self.get_color() # Raw color

    # Divide each channel by the corresponding channel in our white reference
    col = [raw[i]/self.white[i] for i in range(3)]

    # Find the maximum component of our color and normalize each channel in range 0..1
    max_component = max(col[0], max(col[1], col[2]))
    norm = [c/max_component for c in col]

    print(norm)
    return norm
  
  # Use the pico to get the detected color instead
  def get_detected(self):
    self.uart.write(b'd')

    buffer = self.uart.read(length=4)
    detected = ustruct.unpack('<i', buffer)[0]

    return detected
  
  # Tell the pico to reset its detected color
  def reset_detected(self):
    self.uart.write(b'e')