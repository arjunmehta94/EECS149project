import threading


class Drone(object):
    """Class representing the drone object for data from drone."""

    def __init__(self):
        self.acc = [None, None, None]
        self.pitch = None
        self.roll = None
        self.yaw = None
        self.lock = threading.Lock()

    def update_acc(self, ax=None, ay=None, az=None):
        if ax:
            self.acc[0] = ax / 1000.0
        if ay:
            self.acc[1] = ay / 1000.0
        if az:
            self.acc[2] = az / 1000.0

    def update_pitch(self, pitch):
        self.pitch = pitch * 57.2958

    def update_roll(self, roll):
        self.roll = roll * 57.2958

    def update_yaw(self, yaw):
        self.yaw = yaw * 57.2958

    def update_height(self, height):
        self.height = height

    def update_roll_pitch_yaw(self, roll, pitch, yaw):
        self.update_roll(roll)
        self.update_pitch(pitch)
        self.update_yaw(yaw)

    def __str__(self):
        return ' | '.join([str(x) for x in self.acc])

    def __repr__(self):
        return ' | '.join([str(x) for x in self.acc])
