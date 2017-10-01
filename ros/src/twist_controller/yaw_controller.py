from math import atan
from pid import PID

class YawController(object):
    def __init__(self, wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle, gains):
        self.wheel_base = wheel_base
        self.steer_ratio = steer_ratio
        self.min_speed = min_speed
        self.max_lat_accel = max_lat_accel

        self.min_angle = -max_steer_angle
        self.max_angle = max_steer_angle
        self.gains = gains
        self.steering_pid = PID(gains)


    def get_angle(self, radius):
        angle = atan(self.wheel_base / radius) * self.steer_ratio
        return max(self.min_angle, min(self.max_angle, angle))

    def get_steering(self, linear_velocity, angular_velocity, current_velocity, current_angular, dt):
        # angular_velocity = current_velocity * angular_velocity / linear_velocity if abs(linear_velocity) > 0. else 0.


        if abs(current_velocity) > 0.1:
            max_yaw_rate = abs(self.max_lat_accel / current_velocity);
            angular_velocity = max(-max_yaw_rate, min(max_yaw_rate, angular_velocity))

        delta_w = angular_velocity - current_angular

        if current_velocity < self.min_speed:
            return 0.0
        else:
            return self.steering_pid.step(delta_w, dt)
            return self.get_angle(max(current_velocity*1.2, self.min_speed) / angular_velocity) if abs(angular_velocity) > 0. else 0.0;
