# bluetooth code for drone

import sys
import time
import serial
import glob

SERIAL_PORT = '/dev/tty.HC-05-DevB'


def serial_ports():
    """lists serial port names

       :raises EnvironmentError if unknown platform

       :returns list of ports
    """
    if sys.platform.startswith('win'):
        ports = ['COM%s' % (i + 1) for i in range(256)]
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        ports = glob.glob('/dev/tty[A-Za-z]*')
    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.*')
    else:
        raise EnvironmentError('Unsupported platform')
    return ports


def establish_connection():
    """
    Tries to connect with SERIAL_PORT. This is hardcoded
    to be the HC-05

    :raises Exception if unable to connect or if
    SerialException

    :returns the serial connection
    """
    ports = serial_ports()
    time.sleep(1)
    if SERIAL_PORT in ports:
        try:
            serial_connection = serial.Serial(SERIAL_PORT)
            return serial_connection
        except serial.SerialException as e:
            raise Exception(e)
    raise Exception("Could not connect")


def close_connection(serial_connection):
    """
    Closes the serial connection
    """
    if not serial_connection:
        return
    serial_connection.close()
    time.sleep(1)
