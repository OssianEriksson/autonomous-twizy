def cmd_scale(angle,
              speed,
              max_steering_angle,
              max_forward_speed,
              max_reverse_speed):
    angle_scale = abs(angle / max_steering_angle)
    speed_scale = abs(speed / (max_forward_speed if speed > 0
                               else max_reverse_speed))

    return angle_scale, speed_scale
