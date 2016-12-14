import threading
class Drone(object):
    def __init__(self):
        self.acc = [None, None, None]
        #self.height = -1
        #self.throttle = None
        self.pitch = None
        self.roll = None
        self.yaw = None
        self.lock = threading.Lock()

    def update_acc(self, ax=None, ay=None, az=None):
        if ax:
            self.acc[0] = ax
        if ay:
            self.acc[1] = ay
        if az:
            self.acc[2] = az

    def update_throttle(self, throttle):
        self.throttle = throttle

    def update_pitch(self, pitch):
        self.pitch = pitch

    def update_roll(self, roll):
        self.roll = roll

    def update_yaw(self, yaw):
        self.yaw = yaw

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
