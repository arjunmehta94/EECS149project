import time
from constants import PITCH, ROLL, THROTTLE, YAW

def override_pitch(vehicle, value=None):
    vehicle.channels.overrides[PITCH] = value
    time.sleep(0.1)


def override_roll(vehicle, value=None):
    vehicle.channels.overrides[ROLL] = value
    time.sleep(0.1)


def override_throttle(vehicle, value=None):
    vehicle.channels.overrides[THROTTLE] = value
    time.sleep(0.1)


def override_yaw(vehicle, value=None):
    vehicle.channels.overrides[YAW] = value
    time.sleep(0.1)
