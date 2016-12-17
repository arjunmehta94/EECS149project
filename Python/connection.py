from dronekit import connect
from vehicle import DroneVehicle


def connect_vehicle(port, wait_ready=True, baud=57600, vehicle_class=DroneVehicle):
    vehicle = connect(port, wait_ready=wait_ready, baud=baud, vehicle_class=vehicle_class)
    return vehicle


def close_connection_vehicle(vehicle):
    vehicle.disarm_vehicle()
    vehicle.close()
