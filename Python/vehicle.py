from dronekit import VehicleMode, Vehicle
import time
from constants import ALT_HOLD, PITCH, ROLL, YAW, THROTTLE


class RawIMU(object):
    def __init__(self, xacc=None, yacc=None, zacc=None):
        self.xacc = xacc
        self.yacc = yacc
        self.zacc = zacc

    def __str__(self):
        return "RAW_IMU: xacc={},yacc={},zacc={}".format(self.xacc, self.yacc,self.zacc)


class DroneVehicle(Vehicle):

    def __init__(self, *args):
        super(DroneVehicle, self).__init__(*args)
        # Create an Vehicle.raw_imu object with initial values set to None.
        self._raw_imu = RawIMU()
        self.decorator()

    # Create a message listener using the decorator.
        @self.on_message('RAW_IMU')
        def listener(self, name, message):
            """
            The listener is called for messages that contain the string specified in the decorator,
            passing the vehicle, message name, and the message.
            The listener writes the message to the (newly attached) ``vehicle.raw_imu`` object
            and notifies observers.
            """
            self._raw_imu.xacc = message.xacc
            self._raw_imu.yacc = message.yacc
            self._raw_imu.zacc = message.zacc
            # Notify all observers of new message (with new value)
            #   Note that argument `cache=False` by default so listeners
            #   are updated with every new message
            self.notify_attribute_listeners('raw_imu', self._raw_imu)

    @property
    def raw_imu(self):
        return self._raw_imu

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

    def override_pitch(self, value=None):
        self.channels.overrides[PITCH] = value
        time.sleep(0.1)

    def override_roll(self, value=None):
        self.channels.overrides[ROLL] = value
        time.sleep(0.1)

    def override_throttle(self, value=None):
        self.channels.overrides[THROTTLE] = value
        time.sleep(0.1)

    def override_yaw(self, value=None):
        self.channels.overrides[YAW] = value
        time.sleep(0.1)

    def override(self, value=None):
        throttle, roll, pitch, yaw = value
        self.channels.overrides = {PITCH: pitch, ROLL: roll, THROTTLE: throttle, YAW: yaw}
        time.sleep(0.1)

    def clear_channels(self):
        self.channels.overrides = {}
        time.sleep(0.1)
