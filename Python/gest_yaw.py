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
    controller = Leap.Controller()
    print "Leap connected"
    start_time = time.time()
    mapper = interp1d([-90.0, 90.0], [1000, 2000])
    vehicle = connect('com3', wait_ready=True, baud=57600)
    print "Connected"
    vehicle.mode = VehicleMode('STABILIZE')
    vehicle.armed = True
    print "Armed"
    #controller.add_listener(listener)
    while True:
        #print "inside loop"
        if time.time() - start_time > 0.01:
            start_time = time.time()
            if controller.is_connected:
                # print "Controller connected: "
                frame = controller.frame()
                for hand in frame.hands:
                    direction = hand.direction
                    normal = hand.palm_normal
                    yaw = direction.yaw * Leap.RAD_TO_DEG
                    print "Sending yaw: " + str(yaw) + ' mapped to: ' + str(int(mapper(yaw)))
                    vehicle.channels.overrides['4'] = int(mapper(yaw))
            else:
                break
                    
    print "Exit"
    vehicle.armed = False

if __name__ == "__main__":
    main()
