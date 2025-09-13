from pybricks.ev3devices import Motor
from pybricks.hubs import EV3Brick
from pybricks.parameters import Button
from pybricks.tools import wait

BLUE = 1
ORANGE = 2

def calib_steering(m: Motor):
  angle_right = m.run_until_stalled(540, duty_limit=70)
  angle_left = m.run_until_stalled(-540, duty_limit=70)
  m.run_target(90, (angle_left + angle_right)/2)
  m.reset_angle(0)

@micropython.native
def rgb_to_hsv(rgb):
  r_norm, g_norm, b_norm = rgb

  max_c = max(r_norm, g_norm, b_norm)
  min_c = min(r_norm, g_norm, b_norm)
  delta = max_c - min_c

  # Calculate Hue (H)
  if delta == 0:
    h = 0
  elif max_c == r_norm:
    h = (60 * ((g_norm - b_norm) / delta) + 360) % 360
  elif max_c == g_norm:
    h = (60 * ((b_norm - r_norm) / delta) + 120) % 360
  else:  # max_c == b_norm
    h = (60 * ((r_norm - g_norm) / delta) + 240) % 360

  # Calculate Saturation (S)
  if max_c == 0:
    s = 0
  else:
    s = (delta / max_c) * 100

  # Calculate Value (V)
  v = max_c * 100

  return (h, s, v)


def is_color(h, s, v, thresh):
  h_min, h_max, s_min, s_max, v_min, v_max = thresh

  return (h_min <= h <= h_max and
          s_min <= s <= s_max and
          v_min <= v <= v_max)

def is_orange(h, s, v):
  threshold_one = (0, 40, 30, 100, 30, 100)
  threshold_two = (340, 360, 30, 100, 30, 100)
  return is_color(h, s, v, threshold_one) or is_color(h, s, v, threshold_two)

def is_blue(h, s, v):
  default_threshold = (200, 260, 30, 100, 30, 100)
  return is_color(h, s, v, default_threshold)

def get_color(rgb):
  h, s, v = rgb_to_hsv(rgb)

  if is_blue(h, s, v):
    return BLUE
  elif is_orange(h, s, v):
    return ORANGE
  
  return 0

def distance_to_angle(dist_mm: int):
  wheel_diameter = 56   # Wheel has 56 mm diameter
  PI = 3.1415926535   # PI
  gear_ratio = 1.4    # Gear ratio of our drive motor to wheels is 1.4:1
  return int(dist_mm / (wheel_diameter * PI) * gear_ratio * 360)

def wait_for_start():
  brick = EV3Brick()
  while not (Button.RIGHT in brick.buttons.pressed()):
    wait(200)