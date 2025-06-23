def calib_steering(m):
  angle_right = m.run_until_stalled(180, duty_limit=70)
  angle_left = m.run_until_stalled(-180, duty_limit=70)
  m.run_target(90, (angle_left + angle_right)/2)
  m.reset_angle(0)