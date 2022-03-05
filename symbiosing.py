import serial
import io
from os import listdir

DEBUG_TAG = "DEBUG: "


def get_possible_ports():
    return [('/dev/' + path) for path in listdir('/dev/') if path.startswith('cu.usbmodem')]


def open_flowio_ports(paths):
    return {result[0]: result[1]
            for result in
            [try_port(path) for path in paths]
            if result is not None
            }


def try_port(path):
    try:
        ser = serial.Serial(path, baudrate=115200, timeout=1)
        ser.write(b'name?')
        response = ser.readline().decode('ascii')
        print(DEBUG_TAG, "response:", response)
        if response.startswith('FlowIO_'):
            return response.strip(), ser
        else:
            return None
    except serial.SerialException as e:
        print("Exception opening port", path, ":", e)
        ser.close()


def clear_ports(ports_dict: dict):
    for port in ports_dict.values():
        port.close()


if __name__ == '__main__':
    paths = get_possible_ports()
    flowios = open_flowio_ports(paths)
    print(flowios)
    clear_ports(flowios)
