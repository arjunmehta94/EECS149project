from lib.unix import Leap
import time
#from dronekit import connect, VehicleMode
from scipy.interpolate import interp1d

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

def main():
    #listener = GestureListener()
    try:
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
        # mapper = interp1d([-90.0, 90.0], [1000, 2000])
        # vehicle = connect('com7', wait_ready=True, baud=57600)
        # print "Connected"
        # vehicle.mode = VehicleMode('STABILIZE')
        # vehicle.armed = True
        # print "Armed"
        # time.sleep(3)
        #controller.add_listener(listener)
        while True:
            #print "inside loop"
            # if time.time() - start_time > 0.01:
            
            #start_time = time.time()
            #$print(controller.is_paused)
            if controller.is_connected:
                frame = controller.frame()
                if len(frame.hands) > 1:
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
                        #print "Set throttle to 1500"
                    for hand in frame.hands:
                        #print str(hand.is_valid)
                        if hand.is_left:
                            height = low_pass_filter(height, hand.translation(first_frame).y, alpha)
                            print "Filtered Height: " + str(height) + " Original: " + str(hand.translation(first_frame).y)
                        else:
                            direction = hand.direction
                            normal = hand.palm_normal
                            yaw = direction.yaw * Leap.RAD_TO_DEG
                            pitch = low_pass_filter(pitch, direction.pitch * Leap.RAD_TO_DEG, alpha)
                            roll = low_pass_filter(roll, normal.roll * Leap.RAD_TO_DEG, alpha)
                            yaw = low_pass_filter(yaw, direction.yaw * Leap.RAD_TO_DEG, alpha)
                            # pitch, roll, yaw = direction.pitch * Leap.RAD_TO_DEG
                            # normal.roll * Leap.RAD_TO_DEG
                            # direction.yaw * Leap.RAD_TO_DEG
                            #print "Pitch: " + str(pitch) + " Roll: " + str(roll) + " Yaw: " + str(yaw)
                            #print "Sending roll: " + str(roll) + ' mapped to: ' + str(int(mapper(roll)))
                        # vehicle.channels.overrides['1'] = int(mapper(roll))
                        # vehicle.channels.overrides['2'] = int(mapper(pitch))
                        # vehicle.channels.overrides['4'] = int(mapper(yaw))
                        time.sleep(0.1)
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
                        height = low_pass_filter(height, frame.hands[0].translation(first_frame).y, alpha)
                        print "Filtered Height: " + str(height) + " Original: " + str(frame.hands[0].translation(first_frame).y)
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
                    time.sleep(0.1)
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
                # print "Release channels"
                # vehicle.close()
                # print "Close telemetry connection"
                time.sleep(1)
                break
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
    # vehicle.armed = False

if __name__ == "__main__":
    main()
