from lib.windows import Leap
import time
from dronekit import connect, VehicleMode, Vehicle
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
from PID import PID

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
        # self.decorator()

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


class RollPitchYaw(object):
    def __init__(self):
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.ax = 0
        self.ay = 0
        self.az = 0

    def __str__(self):
        return ' | '.join([self.roll, self.pitch, self.yaw, self.ax, self.ay, self.az])

    def __repr__(self):
        return ' | '.join([self.roll, self.pitch, self.yaw, self.ax, self.ay, self.az])

rollpitchyaw = RollPitchYaw()
Throttle_PID = PID(P = 190.0, I = 0.0, D = 0.0 )
Throttle_PID.SetPoint = 0.0
Throttle_PID.setSampleTime(0.028)
Throttle_PID.SetPoint = 0.0

def imu_callback(self, attr_name, value):
    global rollpitchyaw
    rollpitchyaw.ax = value.xacc/1000.0
    rollpitchyaw.ay = value.yacc/1000.0
    rollpitchyaw.az = value.zacc/1000.0

def attitude_callback(self, attr_name, value):
    global rollpitchyaw
    rollpitchyaw.roll = value.roll* 57.2958
    rollpitchyaw.pitch = value.pitch* 57.2958
    rollpitchyaw.yaw = value.yaw* 57.2958
    # if rollpitchyaw.yaw < -179:
    #     if (value.yaw * 57.2958) > 0:
    #         rollpitchyaw.yaw = rollpitchyaw.yaw + (180 - value.yaw* 57.2958)
    #     else:
    #         rollpitchyaw.yaw = rollpitchyaw.yaw - (value.yaw* 57.2958)
    # elif rollpitchyaw.yaw > 179:
    #     if (value.yaw * 57.2958) < 0:
    #         rollpitchyaw.yaw = rollpitchyaw.yaw + (180 + value.yaw* 57.2958)
    #     else:
    #         rollpitchyaw.yaw = rollpitchyaw.yaw + value.yaw
    # else:
    #     rollpitchyaw.yaw = value.yaw* 57.2958


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

from numpy import cos, sin
import numpy as np
def body_to_inertial(roll, pitch, yaw):
    phi = np.deg2rad(roll) # phi
    theta = np.deg2rad(pitch) # theta
    psi = np.deg2rad(yaw) #psi
    
    return np.dot(np.dot(R_yaw(psi),R_pitch(theta)),R_roll(phi))
    
def R_yaw(psi):
    R_yaw = np.zeros((3,3))
    R_yaw[0,0] = cos(psi)
    R_yaw[0,1] = -1*sin(psi)
    R_yaw[1,0] = sin(psi)
    R_yaw[1,1] = cos(psi)
    R_yaw[2,2] = 1
#     print(R_yaw)
    return R_yaw

def R_pitch(theta):
    R_pitch = np.zeros((3,3))
    R_pitch[0,0] = cos(theta)
    R_pitch[0,2] = sin(theta)
    R_pitch[2,0] = -sin(theta)
    R_pitch[2,2] = cos(theta)
    R_pitch[1,1] = 1
    # print(R_pitch)
    return R_pitch

def R_roll(phi):
    R_roll = np.zeros((3,3))
    R_roll[1,1] = cos(phi)
    R_roll[1,2] = -1*sin(phi)
    R_roll[2,1] = sin(phi)
    R_roll[2,2] = cos(phi)
    R_roll[0,0] = 1
#     print(R_roll)
    return R_roll


def get_left_right_hands(frame):
    if frame.hands[0].is_left:
        return [frame.hands[0], frame.hands[1]]
    return [frame.hands[1], frame.hands[0]]

def main():
    global rollpitchyaw
    #listener = GestureListener()
    try:
        single_hand_check = False
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
        right_hand_out_once = False
        left_hand_out_once = False
        print "Leap connected"
        # start_time = time.time()
        Throttle_mapper = interp1d([-150, 150], [-1.5, 0.2])
        rpc_mapper = interp1d(
            [ROLL_PITCH_YAW_MIN, ROLL_PITCH_YAW_MAX], [ROLL_PITCH_YAW_MIN_MAP, ROLL_PITCH_YAW_MAX_MAP]
        )
        throttle_mapper_takeoff = interp1d(
            [THROTTLE_MIN_TAKEOFF, THROTTLE_MAX_TAKEOFF], [THROTTLE_MIN_TAKEOFF_MAP, THROTTLE_MAX_TAKEOFF_MAP]
        )
        throttle_mapper_flight = interp1d(
            [THROTTLE_MIN_FLIGHT, THROTTLE_MAX_FLIGHT], [THROTTLE_MIN_FLIGHT_MAP, THROTTLE_MAX_FLIGHT_MAP]
        )
        vehicle = connect('com3', wait_ready=True, baud=57600, vehicle_class=DroneVehicle)

        print "Connected"
        vehicle.mode = VehicleMode('STABILIZE')
        #vehicle.armed = True
        print "Armed"
        vehicle.add_attribute_listener('attitude', attitude_callback)
        vehicle.add_attribute_listener('raw_imu', imu_callback)
        time.sleep(1)
        print "Please place both your hands above leap motion"
        while len(controller.frame().hands) != 2:
            time.sleep(0.1)
        print "Starting"
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
                    # if height < 0:
                    #     height = 0
                    # elif height > 150:
                    #     height = 150
                    # height = low_pass_filter(height, left_hand.translation(first_frame).y, alpha)
                    height = clip(left_hand.translation(first_frame).y,  -150, 150)

                    
                    # print "Filtered Height: " + str(height) + " Original: " + str(left_hand.translation(first_frame).y)
                    #else:
                    direction = right_hand.direction
                    normal = right_hand.palm_normal

                    ''' Drone Orientation '''
                    drone_roll = rollpitchyaw.roll
                    drone_pitch = rollpitchyaw.pitch
                    drone_yaw = rollpitchyaw.yaw 

                    ''' Drone Acceleration '''
                    body_x = rollpitchyaw.ax
                    body_y = rollpitchyaw.ay
                    body_z = rollpitchyaw.az
                    inertial_acc = np.dot(body_to_inertial(drone_roll, drone_pitch, drone_yaw), np.array([[body_x],[body_y],[body_z]]))
                    #pitch = low_pass_filter(pitch, direction.pitch * Leap.RAD_TO_DEG, alpha)
                    roll_error = -clip((normal.roll * Leap.RAD_TO_DEG)/4, -7, 7) - drone_roll 
                    pitch_error = clip((direction.pitch * Leap.RAD_TO_DEG)/4, -7, 7) - drone_pitch
                    zeroed_yaw = yaw_on_entry - drone_yaw
                    yaw_error = clip((direction.yaw * Leap.RAD_TO_DEG)/6, -5, 5) + zeroed_yaw
                    Throttle_PID.SetPoint = Throttle_mapper(height)

                    #roll = low_pass_filter(roll, normal.roll * Leap.RAD_TO_DEG, alpha)
                    roll = normal.roll * Leap.RAD_TO_DEG
                    pitch = direction.pitch * Leap.RAD_TO_DEG
                    yaw = direction.yaw * Leap.RAD_TO_DEG
                    # yaw = low_pass_filter(yaw, direction.yaw * Leap.RAD_TO_DEG, alpha)
                    # pitch, roll, yaw = direction.pitch * Leap.RAD_TO_DEG
                    # normal.roll * Leap.RAD_TO_DEG
                    # direction.yaw * Leap.RAD_TO_DEG
                    # print "Pitch: " + str(pitch) + " Roll: " + str(roll) + " Yaw: " + str(yaw)


                    Throttle_PID.update(inertial_acc[2,0])
                    ''' Setting Values '''
                    # vehicle.channels.overrides['1'] = clip(1498  + 12*int(roll_error), 989 , 2007)
                    # vehicle.channels.overrides['2'] = clip(1501 -  13*int(pitch_error), 992 , 2010)
                    # vehicle.channels.overrides['4'] = clip(1500 + 8*int(yaw_error), 990 , 2010)
                    vehicle.channels.overrides['3'] = clip(1200 + Throttle_PID.output, 995 , 1420)
                    ''' Prints '''
                    print "Height: " + str(height) + "PWM: " + str(1200 + Throttle_PID.output) + "Z: " + str(inertial_acc[2, 0])
                    # print " Roll: " + str(roll) + " PWM: " + str(vehicle.channels['1']) + " roll eror: " + str(roll_error) + " drone roll: " + str(rollpitchyaw.roll)
                    # print " Pitch: " + str(pitch) + " PWM: " + str(vehicle.channels['2']) + " pitch eror: " + str(pitch_error) + " drone pitch: " + str(rollpitchyaw.pitch)
                    # print " Yaw: " + str(yaw) + " PWM: " + str(vehicle.channels['4']) + " yaw eror: " + str(yaw_error) + " Zeroed yaw: " + str(zeroed_yaw) + " Drone yaw: " + str(rollpitchyaw.yaw )
                    #print " Roll: " + str(drone_roll) + " Pitch: " + str(drone_pitch) + " Yaw: " + str(drone_yaw) 
                    #print " body_x: " + str(body_x) + " inertial_x: " + str(inertial_acc[0,0]) + " body_y: " + str(body_y) + " inertial_y: " + str(inertial_acc[1,0]) + " body_z: " + str(body_z) + " inertial_z: " + str(inertial_acc[2,0])

                    time.sleep(0.028)
                elif len(frame.hands) == 1:
                    # NEED TO TEST THIS PART FOR WHAT TO SEND WHILE ITS CHECKING THIS SHIT
                    if not single_hand_check:
                        count = 0
                        while len(controller.frame().hands) == 1 and count < 6:
                            time.sleep(0.1)
                            count += 1
                        if count == 6:
                            single_hand_check = True
                        else:
                            continue
                    ''' Drone Orientation '''
                    drone_roll = rollpitchyaw.roll
                    drone_pitch = rollpitchyaw.pitch
                    drone_yaw = rollpitchyaw.yaw

                    ''' Drone Acceleration '''
                    body_x = rollpitchyaw.ax
                    body_y = rollpitchyaw.ay
                    body_z = rollpitchyaw.az
                    inertial_acc = np.dot(body_to_inertial(drone_roll, drone_pitch, drone_yaw), np.array([[body_x],[body_y],[body_z]])) 
                    frame = controller.frame()
                    if frame.hands[0].is_left:
                        left_coming_back = False
                        #if not right_coming_back:
                            #height = 0
                            #right_coming_back = True
                        if not first_frame:
                            height = 0
                            print "After coming back left"
                            first_frame = hand_stable(frame, controller)
                            frame = controller.frame()

                        if not right_hand_out_once:
                            vehicle.channels.overrides = {'1': None, '2': None, '4': None}
                            time.sleep(0.028 * 3)
                            right_hand_out_once = True

                        height = clip(frame.hands[0].translation(first_frame).y,  -150, 150)

                        roll_error = - drone_roll 
                        pitch_error = - drone_pitch
                        zeroed_yaw = yaw_on_entry - drone_yaw
                        yaw_error =  zeroed_yaw
                        
                        
                        Throttle_PID.SetPoint = Throttle_mapper(height)
                        Throttle_PID.update(inertial_acc[2,0])
                        vehicle.channels.overrides['3'] = clip(1200 + int(Throttle_PID.output), 995 , 1440)
                        #vehicle.channels.overrides['1'] = clip(1498  + 12*int(roll_error), 989 , 2007)
                        #vehicle.channels.overrides['2'] = clip(1501 -  13*int(pitch_error), 992 , 2010)
                        #vehicle.channels.overrides['4'] = clip(1500 + 4.5*int(yaw_error), 990 , 2010)
                        
                        print "Height: " + str(height) + "PWM: " + str(1200 + int(Throttle_PID.output))  + "Z: " + str(inertial_acc[2, 0])
                        time.sleep(0.028)
                        # vehicle.channels.overrides['3'] = int(throttle_mapper_takeoff(height))
                    else:
                        right_coming_back = False
                        print "Right hand only"

                        if not left_coming_back:
                            #left_coming_back = True
                            first_frame = None
                            roll = 0
                            pitch = 0
                            yaw = 0

                        if not left_hand_out_once:
                            vehicle.channels.overrides = {'3': None}
                            time.sleep(0.028)
                            left_hand_out_once = True

                        for hand in frame.hands:
                            #print len(frame.hands)
                            right_coming_back = False
                            direction = hand.direction
                            normal = hand.palm_normal

                            

                            roll_error = -clip((normal.roll * Leap.RAD_TO_DEG)/4, -7, 7) - drone_roll 
                            pitch_error = clip((direction.pitch * Leap.RAD_TO_DEG)/4, -7, 7) - drone_pitch
                            zeroed_yaw = yaw_on_entry - drone_yaw
                            yaw_error = clip((direction.yaw * Leap.RAD_TO_DEG)/6, -5, 5) + zeroed_yaw

                            Throttle_PID.SetPoint = -1 # may need to set to acceleration on the ground
                            Throttle_PID.update(inertial_acc[2,0])

                            roll = normal.roll * Leap.RAD_TO_DEG
                            pitch = direction.pitch * Leap.RAD_TO_DEG
                            yaw = direction.yaw * Leap.RAD_TO_DEG
                            # print "Pitch: " + str(pitch) + " Roll: " + str(roll) + " Yaw: " + str(yaw)
                            # vehicle.channels.overrides['3'] = clip(1200 + Throttle_PID.output, 995 , 1420)
                            # vehicle.channels.overrides['1'] = clip(1498  + 12*int(roll_error), 989 , 2007)
                            # vehicle.channels.overrides['2'] = clip(1501 -  13*int(pitch_error), 992 , 2010)
                            # vehicle.channels.overrides['4'] = clip(1500 + 4.5*int(yaw_error), 990 , 2010)
                            # print " Yaw: " + str(yaw) + " PWM: " + str(vehicle.channels['4']) + " yaw eror: " + str(yaw_error) + " Zeroed yaw: " + str(zeroed_yaw) + " Drone yaw: " + str(rollpitchyaw.yaw )
                            # time.sleep(0.028)
                    # time.sleep(0.028 * 4)
                else:
                    if not left_coming_back or not right_coming_back:
                        print "Relinquishing control"
                        # vehicle.channels.overrides['3'] = 1300
                        # print "Slowing down motor"
                        # time.sleep(0.1)
                        # vehicle.channels.overrides['1'] = None
                        # vehicle.channels.overrides['2'] = None
                        # vehicle.channels.overrides['3'] = None
                        # vehicle.channels.overrides['4'] = None
                        vehicle.channels.overrides = {}
                       
                        single_hand_check = False
                        # print "Release channels"
                        first_frame = None
                        roll = 0
                        pitch = 0
                        yaw = 0
                        height = 0
                        left_coming_back = True
                        right_coming_back = True
                        yaw_on_entry = 0
                        right_hand_out_once = False
                        left_hand_out_once = False
                    else:
                        # print "should be auto"
                        # ''' Drone Orientation '''
                        # drone_roll = rollpitchyaw.roll
                        # drone_pitch = rollpitchyaw.pitch
                        # drone_yaw = rollpitchyaw.yaw

                        # ''' Drone Acceleration '''
                        # body_x = rollpitchyaw.ax
                        # body_y = rollpitchyaw.ay
                        # body_z = rollpitchyaw.az
                        # inertial_acc = np.dot(body_to_inertial(drone_roll, drone_pitch, drone_yaw), np.array([[body_x],[body_y],[body_z]])) 
                        # roll_error = - drone_roll 
                        # pitch_error = - drone_pitch
                        # zeroed_yaw = yaw_on_entry - drone_yaw
                        # yaw_error =  zeroed_yaw
                        
                        
                        # Throttle_PID.SetPoint = Throttle_mapper(height)
                        # Throttle_PID.update(inertial_acc[2,0])
                        # vehicle.channels.overrides['3'] = clip(1200 + Throttle_PID.output, 995 , 1400)
                        # vehicle.channels.overrides['1'] = clip(1498  + 12*int(roll_error), 989 , 2007)
                        # vehicle.channels.overrides['2'] = clip(1501 -  13*int(pitch_error), 992 , 2010)
                        # vehicle.channels.overrides['4'] = clip(1500 + 4.5*int(yaw_error), 990 , 2010)
                        pass
                    time.sleep(0.028)
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
                time.sleep(.1)
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
