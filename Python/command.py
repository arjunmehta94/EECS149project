import connection
import initialization
import overrides


def main():
    try:
        val = raw_input("Connect? Y/N")
        if val.lower() == 'n':
            print "Exiting"
            return
        port = raw_input("Enter port: ")
        baud = raw_input("Enter baud: (if default press enter)")
        if baud == '':
            vehicle = connection.connect_vehicle(port)
        else:
            vehicle = connection.connect_vehicle(port, int(baud))
        initialization.arm(vehicle)
        while True:
            value = raw_input("Throttle input: (Q to quit)")
            if value.lower() == 'q':
                break
            overrides.override_throttle(vehicle, int(value))
        connection.close_connection_vehicle(vehicle)
    except KeyboardInterrupt:
        print "Exiting"
        return

if __name__ == '__main__':
    main()