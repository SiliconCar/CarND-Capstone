from yaw_controller import YawController
from lowpass import LowPassFilter
from pid import PID
GAS_DENSITY = 2.858
ONE_MPH = 0.44704
MAX_SPEED = 20.0


class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
        self.throttle_pid = PID(kwargs['throttle_gains'])
        self.yaw_control = YawController(kwargs['wheel_base'], kwargs['steer_ratio'],
                                         kwargs['min_speed'], kwargs['max_lat_accel'],
                                         kwargs['max_steer_angle'],
                                         kwargs['steering_gains']
                                         )
        #self.filter = LowPassFilter(0.2,0.1)

    '''
    Params:
    target_v - desired linear velocity
    target_w - desired angular velocity
    current_v - current linear velocity
    dbw_enabled - drive by wire enabled (ignore error in this case)
	'''
    def control(self, target_v, target_w, current_v, dbw_enabled):
        # TODO Return throttle, brake, steer

        throttle = 0.3 if current_v.x < (MAX_SPEED*ONE_MPH) else 0.0
        brake  = 0.0
        steer = current_v.x * self.yaw_control.get_steering(target_v.x, target_w.z, current_v.x)
        if(target_v.x <= 1.0):
            brake = 6.0
            throttle = 0.0
            #steer = 0.0
        #brake = 0.0 if current_v.x < MAX_SPEED else 4.47

        # Need to think of something better than a constant to scale the steering

        #steer = self.filter.filt(steer)
        return throttle, brake, steer
