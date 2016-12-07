from dronekit import connect, VehicleMode
import time
from constants import ALT_HOLD, PITCH, ROLL, YAW, THROTTLE


class DroneVehicle(object):

    def __init__(self, name):
        self.vehicle = None
        self.name = name

    def connect(self, port, baud=57600, wait_ready=True):
        self.vehicle = connect(port, baud=baud, wait_ready=wait_ready)

    def disconnect(self):
        self.vehicle.close()

    def arm_vehicle(self):
        if not self.vehicle.armed:
            self.vehicle.armed = True

    def disarm_vehicle(self):
        if self.vehicle.armed:
            self.vehicle.armed = False

    def set_vehicle_mode(self, mode):
        self.vehicle.mode = VehicleMode(mode)

    def get_vehicle_mode(self):
        return self.vehicle.mode.name

    def arm(self):
        print "Arming motors"
        # Copter should arm in ALT_HOLD mode
        self.set_vehicle_mode(ALT_HOLD)
        print "Vehicle Mode: " + str(self.get_vehicle_mode())
        self.arm_vehicle()
        print "Armed: " + str(self.vehicle.armed)

        # Confirm vehicle armed before attempting to take off
        while not self.vehicle.armed:
            print " Waiting for arming..."
            time.sleep(1)

    def override_channel(self, channel=THROTTLE, value=None, values={}):
        if type(channel) == str:
            self.vehicle.channels.override[channel] = value
        elif values != {}:
            self.vehicle.channels.override = values
        time.sleep(0.1)
