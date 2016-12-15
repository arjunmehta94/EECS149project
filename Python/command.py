from connection import (
    connect_vehicle,
    close_connection_vehicle
)
from leap import LeapMotion
from drone import Drone
import time
import threading
from numpy import clip
#from thread_funcs import leap_update

vehicle = None
leap_obj = LeapMotion()
drone_response_obj = Drone()
from lib.unix import Leap
import time
def leap_update():
    global leap_obj, drone_response_obj
    #print "Executing"
    leap_obj.lock.acquire()
    drone_response_obj.lock.acquire()
    if leap_obj._controller.is_connected:
        leap_obj._frame = leap_obj._controller.frame()
        if len(leap_obj._frame.hands) == 2:
            if leap_obj._left_coming_back:
                leap_obj.height = 0
                leap_obj._left_coming_back = False
                #if not leap_obj._first_frame:
                #print "After coming back"
                leap_obj._first_frame = leap_obj._hand_stable()
                leap_obj._frame = leap_obj._controller.frame()
            if leap_obj._right_coming_back:
                leap_obj._right_coming_back = False
                leap_obj.roll = 0
                leap_obj.pitch = 0
                leap_obj.yaw = 0
            if not leap_obj._first_frame:
                #print "First time"
                leap_obj._hand_stable()
                leap_obj._frame = leap_obj._controller.frame()
                #time.sleep(0.1)
                #print "Set throttle to 1500"
            left_hand, right_hand = leap_obj._get_left_right_hands()
            leap_obj.height = leap_obj._low_pass_filter(leap_obj.height, left_hand.translation(leap_obj._first_frame).y)
            #print "Filtered leap_obj.height: " + str(leap_obj.height) + " Original: " + str(left_hand.translation(leap_obj._first_frame).y)
            direction = right_hand.direction
            normal = right_hand.palm_normal
            # roll_error = clip(normal.roll * Leap.RAD_TO_DEG, -12, 12) - drone_response_obj.roll
            # roll = clip(1498 -  15*int(roll_error), 989 , 2007)
            leap_obj.update_p_control_roll_pitch_yaw(
                normal.roll * Leap.RAD_TO_DEG, drone_response_obj.roll,
                direction.pitch * Leap.RAD_TO_DEG, drone_response_obj.pitch,
                direction.yaw * Leap.RAD_TO_DEG, drone_response_obj.yaw
            )
            # leap_obj.pitch = leap_obj._low_pass_filter(leap_obj.pitch, direction.pitch * Leap.RAD_TO_DEG)
            # leap_obj.roll = leap_obj._low_pass_filter(leap_obj.roll, normal.roll * Leap.RAD_TO_DEG)
            # leap_obj.yaw = leap_obj._low_pass_filter(leap_obj.yaw, direction.yaw * Leap.RAD_TO_DEG)
            #print "leap.pitch: " + str(leap_obj.pitch) + " leap_obj.roll: " + str(leap_obj.roll) + " leap_obj.yaw: " + str(leap_obj.yaw)
            #time.sleep(0.1)
        elif len(leap_obj._frame.hands) == 1:
            leap_obj._frame = leap_obj._controller.frame()
            if leap_obj._frame.hands[0].is_left:
                leap_obj._left_coming_back = False
                if not leap_obj._first_frame:
                    leap_obj.height = 0
                    #print "After coming back"
                    leap_obj._hand_stable()
                    leap_obj._frame = leap_obj._controller.frame()
                leap_obj.height = leap_obj._low_pass_filter(leap_obj.height, leap_obj._frame.hands[0].translation(leap_obj._first_frame).y)
                #print "Filtered leap.height: " + str(leap_obj.height) + " Original: " + str(leap_obj._frame.hands[0].translation(leap_obj._first_frame).y)
            else:
                if not leap_obj._left_coming_back:
                    leap_obj._first_frame = None
                    leap_obj.roll = 0
                    leap_obj.pitch = 0
                    leap_obj.yaw = 0
                hand = leap_obj._frame.hands[0]
                leap_obj._right_coming_back = False
                direction = hand.direction
                normal = hand.palm_normal
                leap_obj.update_p_control_roll_pitch_yaw(
                    normal.roll * Leap.RAD_TO_DEG, drone_response_obj.roll,
                    direction.pitch * Leap.RAD_TO_DEG, drone_response_obj.pitch,
                    direction.yaw * Leap.RAD_TO_DEG, drone_response_obj.yaw
                )
                # leap_obj.pitch = leap_obj._low_pass_filter(leap_obj.pitch, direction.pitch * Leap.RAD_TO_DEG)
                # leap_obj.roll = leap_obj._low_pass_filter(leap_obj.roll, normal.roll * Leap.RAD_TO_DEG)
                # leap_obj.yaw = leap_obj._low_pass_filter(leap_obj.yaw, direction.yaw * Leap.RAD_TO_DEG)
                #print "leap.pitch: " + str(leap_obj.pitch) + " leap.roll: " + str(leap_obj.roll) + " leap.yaw: " + str(leap_obj.yaw)
            #time.sleep(0.1)
        else:
            if not leap_obj._left_coming_back and not leap_obj._right_coming_back:
                #time.sleep(0.1)
                leap_obj._first_frame = None
                leap_obj.roll = 0
                leap_obj.pitch = 0
                leap_obj.yaw = 0
                leap_obj.height = 0
                leap_obj._left_coming_back = True
                leap_obj._right_coming_back = True
    else:
        time.sleep(1)
    drone_response_obj.lock.release()
    leap_obj.lock.release()
    threading.Timer(0.1, leap_update).start()


def imu_callback(self, attr_name, value):
    global drone_response_obj
    drone_response_obj.lock.acquire()
    drone_response_obj.update_acc(value.xacc, value.yacc, value.zacc)
    drone_response_obj.lock.release()

def attitude_callback(self, attr_name, value):
    global drone_response_obj
    drone_response_obj.lock.acquire()
    drone_response_obj.update_roll_pitch_yaw(value.roll, value.pitch, value.yaw)
    drone_response_obj.lock.release()


def main():
    global vehicle, leap_obj, drone_response_obj
    try:
        # create global objs vehicle, leap, drone response

        leap_thread = threading.Timer(0.1, leap_update)

        # todo: add listeners on vehicle for responses

        # todo: initialize and start thread for leap

        # command line inputs and connection, arming phase
        val = raw_input("Connect? Y/N")
        if val.lower() == 'n':
            print "Exiting"
            return
        port = raw_input("Enter port: ")
        #vehicle = connect_vehicle(port)
        print "Connected!"
        time.sleep(0.5)
        #vehicle.add_listener('raw_imu', imu_callback)
        #vehicle.add_listener('attitude', attitude_callback)
        val = raw_input("Arm Y/N")
        if val.lower() == 'n':
            print "Disconnecting, exiting"
            close_connection_vehicle(vehicle)
        #vehicle.arm()
        print "Armed!"
        time.sleep(2)
        leap_thread.start()
        # main loop of flight control
        while True:
            leap_obj.lock.acquire()
            drone_response_obj.lock.acquire()
            print "Leap: " + str(leap_obj)
            print "Drone: " + str(drone_response_obj)
            vehicle.override(leap_obj.get())
            drone_response_obj.lock.release()
            leap_obj.lock.release()
            #print drone_response_obj
            time.sleep(0.1)

        #close_connection_vehicle(vehicle)
    except KeyboardInterrupt:
        #close_connection_vehicle(vehicle)
        print "Exiting"
        return

if __name__ == '__main__':
    main()


