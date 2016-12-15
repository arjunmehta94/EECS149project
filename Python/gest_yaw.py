from lib.windows import Leap
import time
from dronekit import connect, VehicleMode
from scipy.interpolate import interp1d
from constants import (
    ROLL_PITCH_YAW_MIN,
    ROLL_PITCH_YAW_MIN_MAP,
    ROLL_PITCH_YAW_MAX,
    ROLL_PITCH_YAW_MAX_MAP,
    THROTTLE_MIN_TAKEOFF,
    THROTTLE_MAX_TAKEOFF,
    THROTTLE_MAX_TAKEOFF_MAP,
    THROTTLE_MIN_TAKEOFF_MAP,
    THROTTLE_MIN_FLIGHT,
    THROTTLE_MAX_FLIGHT,
    THROTTLE_MAX_FLIGHT_MAP,
    THROTTLE_MIN_FLIGHT_MAP
)
from numpy import clip
import numpy as np
class RollPitchYaw(object):
    def __init__(self):
        self.roll = 0
        self.pitch = 0
        self.yaw = 0

    def __str__(self):
        return ' | '.join([self.roll, self.pitch, self.yaw])

    def __repr__(self):
        return ' | '.join([self.roll, self.pitch, self.yaw])

rollpitchyaw = RollPitchYaw()

def attitude_callback(self, attr_name, value):
    global rollpitchyaw
    rollpitchyaw.roll = value.roll* 57.2958
    rollpitchyaw.pitch = value.pitch* 57.2958
    if rollpitchyaw.yaw < -179:
        if (value.yaw * 57.2958) > 0:
            rollpitchyaw.yaw = rollpitchyaw.yaw + (180 - value.yaw* 57.2958)
        else:
            rollpitchyaw.yaw = rollpitchyaw.yaw - (value.yaw* 57.2958)
    elif rollpitchyaw.yaw > 179:
        if (value.yaw * 57.2958) < 0:
            rollpitchyaw.yaw = rollpitchyaw.yaw + (180 + value.yaw* 57.2958)
        else:
            rollpitchyaw.yaw = rollpitchyaw.yaw + value.yaw
    else:
        rollpitchyaw.yaw = value.yaw* 57.2958


def hand_stable(frame, controller):
    print "Keep your hand flat and stable"
    time.sleep(1)
    first_frame = frame
    print "Calibrating"
    while True:
        frame = controller.frame()
        time.sleep(1)
        if abs(first_frame.translation(frame).y) < 10:
            print "Calibrated frame"
            return frame
        print "Please keep your hand flat and stable"
        first_frame = frame

def low_pass_filter(old, new, alpha):
    return old + alpha * (new - old)

def body_to_inertial(roll, pitch, yaw):


def get_left_right_hands(frame):
    if frame.hands[0].is_left:
        return [frame.hands[0], frame.hands[1]]
    return [frame.hands[1], frame.hands[0]]

def main():
    global rollpitchyaw
    #listener = GestureListener()
    try:
        yaw_on_entry = 0
        left_coming_back = False
        right_coming_back = False
        first_frame = None
        controller = Leap.Controller()
        time.sleep(1)
        roll = 0
        pitch = 0
        yaw = 0
        height = 0
        alpha = 0.2
        print "Leap connected"
        # start_time = time.time()
        rpc_mapper = interp1d(
            [ROLL_PITCH_YAW_MIN, ROLL_PITCH_YAW_MAX], [ROLL_PITCH_YAW_MIN_MAP, ROLL_PITCH_YAW_MAX_MAP]
        )
        throttle_mapper_takeoff = interp1d(
            [THROTTLE_MIN_TAKEOFF, THROTTLE_MAX_TAKEOFF], [THROTTLE_MIN_TAKEOFF_MAP, THROTTLE_MAX_TAKEOFF_MAP]
        )
        throttle_mapper_flight = interp1d(
            [THROTTLE_MIN_FLIGHT, THROTTLE_MAX_FLIGHT], [THROTTLE_MIN_FLIGHT_MAP, THROTTLE_MAX_FLIGHT_MAP]
        )
        vehicle = connect('com3', wait_ready=True, baud=57600)

        print "Connected"
        vehicle.mode = VehicleMode('STABILIZE')
        vehicle.armed = True
        print "Armed"
        vehicle.add_attribute_listener('attitude', attitude_callback)
        time.sleep(1)
        #controller.add_listener(listener)
        while True:
            #print "inside loop"
            # if time.time() - start_time > 0.01:
            
            #start_time = time.time()
            #$print(controller.is_paused)
            if controller.is_connected: #and vehicle.location.global_relative_frame.alt > 0.3:
                frame = controller.frame()
                if len(frame.hands) == 2:
                    if left_coming_back:
                        height = 0
                        left_coming_back = False
                        #if not first_frame:
                        print "After coming back"
                        first_frame = hand_stable(frame, controller)
                        frame = controller.frame()
                    if right_coming_back:
                        right_coming_back = False
                        roll = 0
                        pitch = 0
                        yaw = 0
                    if not first_frame:
                        print "First time"
                        first_frame = hand_stable(frame, controller)
                        frame = controller.frame()
                        time.sleep(0.1)
                        yaw_on_entry = rollpitchyaw.yaw
                        #print "Set throttle to 1500"
                    left_hand, right_hand = get_left_right_hands(frame)
                    #for hand in frame.hands:
                        #print str(hand.is_valid)
                    #if hand.is_left:
                    if height < 0:
                        height = 0
                    elif height > 150:
                        height = 150
                    height = low_pass_filter(height, left_hand.translation(first_frame).y, alpha)
                    
                    # print "Filtered Height: " + str(height) + " Original: " + str(left_hand.translation(first_frame).y)
                    #else:
                    direction = right_hand.direction
                    normal = right_hand.palm_normal
                    #pitch = low_pass_filter(pitch, direction.pitch * Leap.RAD_TO_DEG, alpha)
                    roll_error = -clip((normal.roll * Leap.RAD_TO_DEG)/4, -7, 7) - rollpitchyaw.roll 
                    pitch_error = clip((direction.pitch * Leap.RAD_TO_DEG)/4, -7, 7) - rollpitchyaw.pitch
                    zeroed_yaw = yaw_on_entry - rollpitchyaw.yaw 
                    yaw_error = clip((direction.yaw * Leap.RAD_TO_DEG)/6, -5, 5) + zeroed_yaw
                    #roll = low_pass_filter(roll, normal.roll * Leap.RAD_TO_DEG, alpha)
                    roll = normal.roll * Leap.RAD_TO_DEG
                    pitch = direction.pitch * Leap.RAD_TO_DEG
                    yaw = direction.yaw * Leap.RAD_TO_DEG
                    # yaw = low_pass_filter(yaw, direction.yaw * Leap.RAD_TO_DEG, alpha)
                    # pitch, roll, yaw = direction.pitch * Leap.RAD_TO_DEG
                    # normal.roll * Leap.RAD_TO_DEG
                    # direction.yaw * Leap.RAD_TO_DEG
                    # print "Pitch: " + str(pitch) + " Roll: " + str(roll) + " Yaw: " + str(yaw)
                    ''' Setting Values '''
                    #vehicle.channels.overrides['1'] = clip(1498  + 12*int(roll_error), 989 , 2007)
                    # vehicle.channels.overrides['2'] = clip(1501 -  13*int(pitch_error), 992 , 2010)
                    vehicle.channels.overrides['4'] = clip(1500 + 4.5*int(yaw_error), 990 , 2010)
                    ''' Prints '''
                    #print " Roll: " + str(roll) + " PWM: " + str(vehicle.channels['1']) + " roll eror: " + str(roll_error) + " drone roll: " + str(rollpitchyaw.roll)
                    # print " Pitch: " + str(pitch) + " PWM: " + str(vehicle.channels['2']) + " pitch eror: " + str(pitch_error) + " drone pitch: " + str(rollpitchyaw.pitch)
                    print " Yaw: " + str(yaw) + " PWM: " + str(vehicle.channels['4']) + " yaw eror: " + str(yaw_error) + " Zeroed yaw: " + str(zeroed_yaw) + " Drone yaw: " + str(rollpitchyaw.yaw )
                    # vehicle.channels.overrides['2'] = int(rpc_mapper(pitch))
                    # vehicle.channels.overrides['4'] = int(rpc_mapper(yaw))
                    # vehicle.channels.overrides['3'] = int(throttle_mapper_takeoff(height))
                    time.sleep(0.028)
                elif len(frame.hands) == 1:
                    frame = controller.frame()
                    if frame.hands[0].is_left:
                        left_coming_back = False
                        #if not right_coming_back:
                            #height = 0
                            #right_coming_back = True
                        if not first_frame:
                            height = 0
                            print "After coming back"
                            first_frame = hand_stable(frame, controller)
                            frame = controller.frame()
                        #right_coming_back = True
                        #import pdb; pdb.set_trace()
                        if height < 0:
                            height = 0
                        elif height > 150:
                            height = 150
                        height = low_pass_filter(height, frame.hands[0].translation(first_frame).y, alpha)
                        
                        print "Filtered Height: " + str(height) + " Original: " + str(frame.hands[0].translation(first_frame).y)
                        # vehicle.channels.overrides['3'] = int(throttle_mapper_takeoff(height))
                    else:
                        #right_coming_back = False
                        if not left_coming_back:
                            #left_coming_back = True
                            first_frame = None
                            roll = 0
                            pitch = 0
                            yaw = 0
                        for hand in frame.hands:
                            right_coming_back = False
                            direction = hand.direction
                            normal = hand.palm_normal
                            yaw = direction.yaw * Leap.RAD_TO_DEG
                            pitch = low_pass_filter(pitch, direction.pitch * Leap.RAD_TO_DEG, alpha)
                            roll = low_pass_filter(roll, normal.roll * Leap.RAD_TO_DEG, alpha)
                            yaw = low_pass_filter(yaw, direction.yaw * Leap.RAD_TO_DEG, alpha)
                            print "Pitch: " + str(pitch) + " Roll: " + str(roll) + " Yaw: " + str(yaw)
                            #vehicle.channels.overrides['1'] = int(rpc_mapper(roll))
                            # vehicle.channels.overrides['2'] = int(rpc_mapper(pitch))
                            # vehicle.channels.overrides['4'] = int(rpc_mapper(yaw))
                            
                    time.sleep(0.028)
                else:
                    if not left_coming_back and not right_coming_back:
                        # print "Relinquishing control"
                        # vehicle.channels.overrides['3'] = 1300
                        # print "Slowing down motor"
                        # time.sleep(0.1)
                        # vehicle.channels.overrides['1'] = None
                        # vehicle.channels.overrides['2'] = None
                        # vehicle.channels.overrides['3'] = None
                        # vehicle.channels.overrides['4'] = None
                        vehicle.channels.overrides = {}
                        # print "Release channels"
                        time.sleep(0.1)
                        first_frame = None
                        roll = 0
                        pitch = 0
                        yaw = 0
                        height = 0
                        left_coming_back = True
                        right_coming_back = True
            else:
                # print "Relinquishing control"
                # vehicle.channels.overrides['3'] = 1300
                # print "Slowing down motor"
                # time.sleep(0.1)
                # vehicle.channels.overrides['1'] = None
                # vehicle.channels.overrides['2'] = None
                # vehicle.channels.overrides['3'] = None
                # vehicle.channels.overrides['4'] = None
                vehicle.channels.overrides = {}
                # print "Release channels"
                #vehicle.close()
                # print "Close telemetry connection"
                time.sleep(.028)
                #break
    except KeyboardInterrupt, ValueError:
        # print "Relinquishing control"
        # vehicle.channels.overrides['1'] = None
        # vehicle.channels.overrides['2'] = None
        # vehicle.channels.overrides['3'] = None
        # vehicle.channels.overrides['4'] = None
        # time.sleep(1)
        # vehicle.armed = False
        # while vehicle.armed:
        #      print "Disarming"
        # time.sleep(1)
        # vehicle.close()
        # print "Interrupted"
        return
                    
    print "Exit"
    vehicle.close()
    # vehicle.armed = False

if __name__ == "__main__":
    main()
