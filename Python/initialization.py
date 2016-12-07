from vehicle import DroneVehicle


def init(port, baud=57600):
    vehicle = DroneVehicle("EECS 149/249")
    vehicle.connect(port, baud=baud)
    vehicle.arm()
    return vehicle


def close(vehicle):
    vehicle.disarm_vehicle()
    vehicle.close()
