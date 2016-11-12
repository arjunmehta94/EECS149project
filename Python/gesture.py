from lib import Leap
from bluetooth import (
    establish_connection, close_connection
)
import sys


class GestureListener(Leap.Listener):

    def on_init(self, controller):
        self.serial_connection = establish_connection()
        print "Initialized"

    def on_connect(self, controller):
        print "Connected"

    def on_disconnect(self, controller):
        print "Disconnected"

    def on_exit(self, controller):
        close_connection(self.serial_connection)
        print "Exit"

    def on_frame(self, controller):
        frame = controller.frame()
        if not frame.hands:
            return
        for hand in frame.hands:
            direction = hand.direction
            normal = hand.palm_normal
            send_string = "pitch %f roll %f yaw %f\n" % (
                direction.pitch * Leap.RAD_TO_DEG,
                normal.roll * Leap.RAD_TO_DEG,
                direction.yaw * Leap.RAD_TO_DEG
            )
            print(send_string)
            self.serial_connection.write(send_string)


def main():
    listener = GestureListener()
    controller = Leap.Controller()
    controller.add_listener(listener)
    try:
        sys.stdin.readline()
    except KeyboardInterrupt:
        pass
    finally:
        # Remove the sample listener when done
        controller.remove_listener(listener)

if __name__ == "__main__":
    main()
