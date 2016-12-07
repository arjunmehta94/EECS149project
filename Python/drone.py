class Drone(object):
    def __init__(self):
        self.acc = [None, None, None]
        self.height = -1
        self.throttle = None
        self.pitch = None
        self.roll = None
        self.yaw = None

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
