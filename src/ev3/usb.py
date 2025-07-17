import ffi

class USB():
  def __init__(self, port=b'/dev/ttyACM0', baudrate=115200):
    lib = ffi.open('serial_ev3/libusbserial.so')

    self.init_usb = lib.func('i', 'init_usb', 'si')
    self.reset_input_buffer = lib.func('v', 'reset_input_buffer', '')
    self.write_bytes = lib.func('i', 'write_bytes', 'pi')
    self.read_bytes = lib.func('i', 'read_bytes', 'pi')
    self.deinit_usb = lib.func('v', 'deinit_usb', '')

    self.init_usb(port, baudrate)
    self.reset_input_buffer()
