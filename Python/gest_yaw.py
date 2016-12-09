from lib import Leap
from bluetooth import (
    establish_connection, close_connection
)
import sys
import time
from dronekit import connect, VehicleMode
from scipy.interpolate import interp1d

def main():
    #listener = GestureListener()
    try:
        relinquished = False
        first_frame = None
        controller = Leap.Controller()
        time.sleep(1)
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
                if len(frame.hands) > 0:
                    if relinquished:
                        relinquished = False
                    else:
                        #vehicle.channels.overrides['3'] = 1500
                        if not first_frame:
                            first_frame = frame
                        time.sleep(0.1)
                        #print "Set throttle to 1500"
                    for hand in frame.hands:
                        #print str(hand.is_valid)
                        print str(hand.translation(first_frame).y)
                        direction = hand.direction
                        normal = hand.palm_normal
                        yaw = direction.yaw * Leap.RAD_TO_DEG
                        pitch, roll, yaw = direction.pitch * Leap.RAD_TO_DEG, normal.roll * Leap.RAD_TO_DEG, direction.yaw * Leap.RAD_TO_DEG
                        #print "Sending roll: " + str(roll) + ' mapped to: ' + str(int(mapper(roll)))
                        # vehicle.channels.overrides['1'] = int(mapper(roll))
                        # vehicle.channels.overrides['2'] = int(mapper(pitch))
                        # vehicle.channels.overrides['4'] = int(mapper(yaw))
                        time.sleep(0.1)
                else:
                    if not relinquished:
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
                        relinquished = True
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
