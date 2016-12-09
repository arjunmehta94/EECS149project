import initialization
from dronekit import connect


def connect_vehicle(port, wait_ready=True, baud=57600):
    vehicle = connect(port, wait_ready=wait_ready, baud=baud)
    return vehicle


def close_connection_vehicle(vehicle):
    initialization.disarm_vehicle(vehicle)
    vehicle.close()
