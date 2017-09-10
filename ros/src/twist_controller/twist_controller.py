
GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
        pass

    '''
    Params:
    v - desired linear velocity
    w - desired angular velocity
    current_v - current linear velocity
    dbw_enabled - drive by wire enabled (ignore error in this case)
	'''
    def control(self, v, w, current_v, dbw_enabled):
        # TODO Return throttle, brake, steer

        return .3, 0., 0.
