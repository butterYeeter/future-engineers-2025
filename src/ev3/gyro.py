import ffi
import ustruct

lib_path = './serial_ev3/libusbserial.so'

class Gyro():
  def __init__(self):
    serial = ffi.open(lib_path)

    self.init_usb = serial.func('i', 'init_usb', 'si')
    self.reset_input_buffer = serial.func('v', 'reset_input_buffer', '')
    self.write_bytes = serial.func('i', 'write_bytes', 'pi')
    self.read_bytes = serial.func('i', 'read_bytes', 'pi')
    self.deinit_usb = serial.func('v', 'deinit_usb', '')

  def init(self, port=b'/dev/ttyACM0', baudrate=115200):
    self.init_usb(port, baudrate)
    self.reset_input_buffer()

  def get_angle(self):
    cmd = b'a'
    self.write_bytes(cmd, len(cmd))

    buffer = bytearray(32)
    self.read_bytes(buffer, len(buffer))

    return ustruct.unpack('f', buffer)[0]
  
  def reset_angle(self):
    cmd = b'r'

    self.write_bytes(cmd, len(cmd))
  
  def calibrate(self, num_samples):
    cmd = bytearray(2)
    cmd.append(b'g')
    cmd.append(num_samples)

    self.write_bytes(cmd, len(cmd))



  def deinit(self):
    self.deinit_usb()