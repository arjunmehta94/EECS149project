import time
from scipy.interpolate import interp1d
from constraints import MIN_MAX_RC, MIN_MAX_LEAP
from numpy import clip

def map_pitch(x):
	m = (MIN_MAX_RC[0][1]-MIN_MAX_RC[0][0])/(MIN_MAX_LEAP[0][1]- MIN_MAX_LEAP[0][0])
	b = (MIN_MAX_RC[0][1]-MIN_MAX_RC[0][0])/2
	return clip(m*x+b, MIN_MAX_RC[0][0], MIN_MAX_RC[0][1])
    # return int(interp1d([, ], [MIN_MAX_RC[0][0], MIN_MAX_RC[0][1]]))


def map_roll():
    vehicle.channels.overrides[ROLL] = value

def map_throttle():
    vehicle.channels.overrides[THROTTLE] = value

def map_yaw():
    vehicle.channels.overrides[YAW] = value
