from pybricks.tools import DataLog

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

# import enum

def is_color(h, s, v, thresh):
  h_min, h_max, s_min, s_max, v_min, v_max = thresh

  return (h_min <= h <= h_max and
          s_min <= s <= s_max and
          v_min <= v <= v_max)

def is_orange(rgb, log: DataLog):
  h, s, v = rgb_to_hsv(rgb)
  default_threshold = (0, 50, 40, 100, 40, 100)
  # log.log(rgb, (h,s,v))
  return is_color(h, s, v, default_threshold)

def is_blue(rgb, log: DataLog):
  h, s, v = rgb_to_hsv(rgb)
  default_threshold = (200, 260, 40, 100, 40, 100)
  # log.log(rgb, (h,s,v))
  return is_color(h, s, v, default_threshold)
