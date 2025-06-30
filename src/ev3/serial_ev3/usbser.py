import ffi

libusbser
usbinit
usbflushinput
usbgetangle
usbdeinit

def usbser_init(path_to_lib):
  libusbser = ffi.open(path_to_lib)

  usbinit = libusbser.func('i', 'init_usb', 'si')
  usbflushinput = libusbser.func('v', 'flush_input', '')
  subgetangle = libusbser.func('f', 'angle', '')
  usbdeinit = libusbser.func('v', 'deinit', '')