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
        self.last_t = None
        #self.filter = LowPassFilter(0.2,0.1)

    '''
    Params:
    target_v - desired linear velocity
    target_w - desired angular velocity
    current_v - current linear velocity
    dbw_enabled - drive by wire enabled (ignore error in this case)
	'''
    def control(self, target_v, target_w, current_v, dbw_enabled):
        # Get throttle value from controller
        if self.last_t is None:
            self.last_t = time.time()

        dt = time.time() - self.last_t
        target_v = min(target_v, MAX_SPEED*ONE_MPH)
        error_v = target_v - current_v
        throttle = self.throttle_pid.step(error_v, dt)

        if error_v < -1:
            brake  = -3.0*error_v   # Proportional braking

        steer = current_v.x * self.yaw_control.get_steering(target_v.x, target_w.z, current_v.x)
        # if(target_v.x <= 1.0):
        #     brake = 6.0
        #     throttle = 0.0
        #     #steer = 0.0

        #steer = self.filter.filt(steer)
        self.last_t = time.time()
        return throttle, brake, steer
