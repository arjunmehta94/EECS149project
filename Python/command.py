from connection import (
    connect_vehicle,
    close_connection_vehicle
)
from leap import LeapMotion
from drone import Drone
import time
import threading
from numpy import clip


# global objects to work with
vehicle = None
leap_obj = LeapMotion()
drone_response_obj = Drone()
from lib.unix import Leap
import time


def leap_update():
    """Callback function for the leap thread."""
    global leap_obj, drone_response_obj
    leap_obj.lock.acquire()
    if leap_obj._controller.is_connected:
        leap_obj._frame = leap_obj._controller.frame()

        # 2 hands above leap
        if len(leap_obj._frame.hands) == 2:
            if leap_obj._left_coming_back:
                leap_obj.height = 0
                leap_obj._left_coming_back = False
                leap_obj._first_frame = leap_obj._hand_stable()
                leap_obj._frame = leap_obj._controller.frame()
            if leap_obj._right_coming_back:
                leap_obj._right_coming_back = False
                leap_obj.roll = 0
                leap_obj.pitch = 0
                leap_obj.yaw = 0
            if not leap_obj._first_frame:
                leap_obj._hand_stable()
                leap_obj._frame = leap_obj._controller.frame()
                leap.obj.yaw_on_entry = rollpitchyaw.yaw

            left_hand, right_hand = leap_obj._get_left_right_hands()
            height = clip(left_hand.translation(first_frame).y,  -150, 150)

            # acquire updated values from drone object
            drone_response_obj.lock.acquire()
            drone_roll, drone_pitch, drone_yaw = drone_response_obj.roll, drone_response_obj.pitch, drone_response_obj.yaw
            body_x, body_y, body_z = drone_response_obj.ax, drone_response_obj.ay, drone_response_obj.az
            drone_response_obj.lock.release()

            inertial_acc = np.dot(body_to_inertial(drone_roll, drone_pitch, drone_yaw), np.array([[body_x],[body_y],[body_z]]))
            direction = right_hand.direction
            normal = right_hand.palm_normal
            leap_obj.Throttle_PID.SetPoint = Throttle_mapper(height)
            leap_obj.Throttle_PID.update(inertial_acc[2,0])

            # update leap obj with correct values
            leap_obj.update_p_control_throttle()
            leap_obj.update_p_control_roll_pitch_yaw(
                normal.roll * Leap.RAD_TO_DEG, drone_roll,
                direction.pitch * Leap.RAD_TO_DEG, drone_pitch,
                direction.yaw * Leap.RAD_TO_DEG, leap_obj.yaw_on_entry - drone_yaw
            )

        # 1 hand above leap
        elif len(leap_obj._frame.hands) == 1:

            # does a timed check on 1 hand being present to prevent hysteresis
            if not leap_ob._single_hand_check:
                count = 0
                while len(controller.frame().hands) == 1 and count < 6:
                    time.sleep(0.1)
                    count += 1
                if count == 6:
                    leap_obj._single_hand_check = True
                else:
                    continue
            leap_obj._frame = leap_obj._controller.frame()

            # acquire updated values from drone object
            drone_response_obj.lock.acquire()
            drone_roll, drone_pitch, drone_yaw = drone_response_obj.roll, drone_response_obj.pitch, drone_response_obj.yaw
            body_x, body_y, body_z = drone_response_obj.ax, drone_response_obj.ay, drone_response_obj.az
            drone_response_obj.lock.release()

            if leap_obj._frame.hands[0].is_left:
                leap_obj.num_channels_overriden = 1
                leap_obj._left_coming_back = False
                if not leap_obj._first_frame:
                    leap_obj.height = 0
                    leap_obj._hand_stable()
                    leap_obj._frame = leap_obj._controller.frame()
                inertial_acc = np.dot(body_to_inertial(drone_roll, drone_pitch, drone_yaw), np.array([[body_x],[body_y],[body_z]]))
                leap_obj.Throttle_PID.SetPoint = Throttle_mapper(height)
                leap_obj.Throttle_PID.update(inertial_acc[2,0])

                # update leap obj with correct values
                leap_obj.update_p_control_throttle()
            else:
                leap_obj.num_channels_overriden = 3
                if not leap_obj._left_coming_back:
                    leap_obj._first_frame = None
                    leap_obj.roll = 0
                    leap_obj.pitch = 0
                    leap_obj.yaw = 0
                hand = leap_obj._frame.hands[0]
                leap_obj._right_coming_back = False
                direction = hand.direction
                normal = hand.palm_normal

                # update leap obj with correct values
                leap_obj.update_p_control_roll_pitch_yaw(
                    normal.roll * Leap.RAD_TO_DEG, drone_roll,
                    direction.pitch * Leap.RAD_TO_DEG, drone_pitch,
                    direction.yaw * Leap.RAD_TO_DEG, leap_obj.yaw_on_entry - drone_yaw
                )

        # no hands above leap
        else:
            if not leap_obj._left_coming_back or not leap_obj._right_coming_back:
                # reset sequence, does this once every time both hands removed
                leap_obj.num_channels_overriden = 4
                leap_obj._single_hand_check = False
                leap_obj._first_frame = None
                leap_obj.roll = 0
                leap_obj.pitch = 0
                leap_obj.yaw = 0
                leap_obj.height = 0
                leap_obj.yaw_on_entry = 0
                leap_obj._left_coming_back = True
                leap_obj._right_coming_back = True
                leap_obj.clear_channels()
            else:
                # auto mode, when no hands present (can substitue with relinquishing controls to RC)
                # should hover in fixed position and height

                # acquire updated values from drone object
                drone_response_obj.lock.acquire()
                drone_roll, drone_pitch, drone_yaw = drone_response_obj.roll, drone_response_obj.pitch, drone_response_obj.yaw
                body_x, body_y, body_z = drone_response_obj.ax, drone_response_obj.ay, drone_response_obj.az
                drone_response_obj.lock.release()
                leap_obj.Throttle_PID.SetPoint = -1

                # update leap obj with correct values
                leap.obj.Throttle_PID.update(inertial_acc[2,0])
                leap_obj.update_p_control_roll_pitch_yaw(
                    0, drone_roll,
                    0, drone_pitch,
                    -leap_obj.yaw_on_entry, leap_obj.yaw_on_entry - drone_yaw
                )
    else:
        time.sleep(1)
        leap_obj.clear_channels()
    leap_obj.lock.release()
    threading.Timer(0.1, leap_update).start()


def imu_callback(self, attr_name, value):
    """Listener Callback function for raw acceleration from Drone."""
    global drone_response_obj
    drone_response_obj.lock.acquire()
    drone_response_obj.update_acc(value.xacc, value.yacc, value.zacc)
    drone_response_obj.lock.release()


def attitude_callback(self, attr_name, value):
    """Listener Callback function for attitude from Drone."""
    global drone_response_obj
    drone_response_obj.lock.acquire()
    drone_response_obj.update_roll_pitch_yaw(value.roll, value.pitch, value.yaw)
    drone_response_obj.lock.release()


def main():
    """Main thread for flight control."""
    global vehicle, leap_obj, drone_response_obj
    try:

        # initialize and start thread for leap
        leap_thread = threading.Timer(0.1, leap_update)

        # command line inputs and connection, arming phase
        val = raw_input("Connect? Y/N")
        if val.lower() == 'n':
            print "Exiting"
            return
        port = raw_input("Enter port: ")
        vehicle = connect_vehicle(port)
        print "Connected!"
        time.sleep(0.5)

        # add listeners on vehicle for responses
        vehicle.add_listener('raw_imu', imu_callback)
        vehicle.add_listener('attitude', attitude_callback)

        val = raw_input("Arm Y/N")
        if val.lower() == 'n':
            print "Disconnecting, exiting"
            close_connection_vehicle(vehicle)
        vehicle.arm()
        print "Armed!"
        time.sleep(2)
        print "Please place both your hands above leap motion"
        while len(leap_obj._controller.frame().hands) != 2:
            time.sleep(0.1)
        print "Starting"
        time.sleep(1)
        leap_thread.start()

        # main loop of flight control
        while True:
            leap_obj.lock.acquire()
            drone_response_obj.lock.acquire()
            vehicle.override(leap_obj.get())
            drone_response_obj.lock.release()
            leap_obj.lock.release()
            time.sleep(0.028 * leap_obj.num_channels_overriden)

        close_connection_vehicle(vehicle)
    except KeyboardInterrupt:
        close_connection_vehicle(vehicle)
        print "Exiting"
        return

if __name__ == '__main__':
    main()


