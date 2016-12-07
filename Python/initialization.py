from dronekit import VehicleMode
import time


def arm_vehicle(vehicle):
    vehicle.armed = True
    return vehicle.armed


def disarm_vehicle(vehicle):
    vehicle.armed = False
    return vehicle.armed


def set_vehicle_mode(vehicle, mode):
    """
    Sets VehicleMode to mode
    """
    vehicle.mode = VehicleMode(mode)
    return vehicle.mode.name


def arm(vehicle):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print "Arming motors"
    # Copter should arm in ALT_HOLD mode
    print "Vehicle mode: " + str(set_vehicle_mode("ALT_HOLD"))
    print "Armed: " + str(arm_vehicle())

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print " Waiting for arming..."
        time.sleep(1)


