import time
from scipy.interpolate import interp1d
from constraints import MIN_MAX_RC, MIN_MAX_LEAP

def map_pitch():
    return int(interp1d([, ], [MIN_MAX_RC[0][0], MIN_MAX_RC[0][1]]))


def map_roll():
    vehicle.channels.overrides[ROLL] = value

def map_throttle():
    vehicle.channels.overrides[THROTTLE] = value

def map_yaw():
    vehicle.channels.overrides[YAW] = value
