from constants import (
    MIN_THROTTLE,
    ALPHA,
    ROLL_ZERO,
    PITCH_ZERO,
    YAW_ZERO,
    ROLL_EXTREME,
    PITCH_EXTREME,
    YAW_EXTREME,
    ROLL_MIN_DRONE,
    ROLL_MAX_DRONE,
    PITCH_MIN_DRONE,
    PITCH_MAX_DRONE,
    YAW_MIN_DRONE,
    YAW_MAX_DRONE,
    KP
)
from lib.unix import Leap
import time
import threading
from numpy import clip


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

    def update_p_control_throttle(self, throttle, throttle_drone):
        throttle_error = clip(throttle, -THROTTLE_EXTREME, THROTTLE_EXTREME) - throttle_drone
        throttle = clip(THROTTLE_ZERO - KP * int(throttle_error), THROTTLE_MIN_DRONE, THROTTLE_MAX_DRONE)
        self.height = throttle

    def update_p_control_roll(self, roll, roll_drone):
        roll_error = clip(roll, -ROLL_EXTREME, ROLL_EXTREME) - roll_drone
        roll = clip(ROLL_ZERO - KP * int(roll_error), ROLL_MIN_DRONE, ROLL_MAX_DRONE)
        self.roll = roll

    def update_p_control_pitch(self, pitch, pitch_drone):
        pitch_error = clip(pitch, -PITCH_EXTREME, PITCH_EXTREME) - pitch_drone
        pitch = clip(PITCH_ZERO - KP * int(pitch_error), PITCH_MIN_DRONE, PITCH_MAX_DRONE)
        self.pitch = pitch

    def update_p_control_yaw(self, yaw, yaw_drone):
        yaw_error = clip(yaw, -PITCH_EXTREME, PITCH_EXTREME) - yaw_drone
        yaw = clip(YAW_ZERO - KP * int(yaw_error), YAW_MIN_DRONE, YAW_MAX_DRONE)
        self.yaw = yaw

    def update_p_control_roll_pitch_yaw(self, roll, roll_drone, pitch, pitch_drone, yaw, yaw_drone):
        self.update_p_control_roll(roll, roll_drone)
        self.update_p_control_pitch(pitch, pitch_drone)
        self.update_p_control_yaw(yaw, yaw_drone)

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

