from constants import (
    MIN_THROTTLE,
    ALPHA
)
from lib.windows import Leap
import time
import threading


class LeapMotion(object):
    def __init__(self):
        self.height = 0
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self._left_coming_back = False
        self._right_coming_back = False
        self._first_frame = None
        self._controller = Leap.Controller()
        self._frame = None
        self.lock = threading.Lock()

    def update_roll_pitch_yaw(self, roll, pitch, yaw):
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw

    def update_throttle(self, height):
        self.height = height

    def update(self, height, roll, pitch, yaw):
        self.height = height
        self.update_roll_pitch_yaw(roll, pitch, yaw)

    def get(self):
        return [self.height, self.roll, self.pitch, self.yaw]

    def _hand_stable(self):
        print "Keep your hand flat and stable"
        time.sleep(0.5)
        self._first_frame = self._frame
        print "Calibrating"
        while True:
            self._frame = self._controller.frame()
            time.sleep(0.5)
            if abs(self._first_frame.translation(self._frame).y) < 10:
                print "Calibrated frame"
                return
            print "Please keep your hand flat and stable"
            self._first_frame = self._frame

    def _low_pass_filter(self, old, new, alpha=ALPHA):
        return old + alpha * (new - old)

    def _get_left_right_hands(self):
        frame = self._frame
        if frame.hands[0].is_left:
            return [frame.hands[0], frame.hands[1]]
        return [frame.hands[1], frame.hands[0]]

    def __str__(self):
        return ' | '.join([str(x) for x in self.get()])

    def __repr__(self):
        return ' | '.join([str(x) for x in self.get()])

