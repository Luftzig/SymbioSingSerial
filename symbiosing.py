import argparse
import json
import time
from collections import namedtuple
from os import listdir
from typing import Literal, Union, Optional, Mapping, Any, Iterable

import serial
import asyncio

DEBUG_TAG = "DEBUG: "
DEBUG = False


def debug(*args, **kwargs):
    if DEBUG:
        print("DEBUG:", *args, **kwargs)


ActionReadable = Literal['stop',
                         'inflate',
                         'vacuum',
                         'release',
                         'actuate']

ActionChar = Literal['!', '+', '-', '^', 'a']

Connected = namedtuple('connected', ['name', 'series', 'role', 'n_timeout'])


def _parse_name(response: str):
    [_, keep] = response.rsplit('_', 1)
    if len(keep) == 2:
        return keep[0], keep[1]
    else:
        return None, None


def _convert_action(action):
    mapping: Mapping[ActionReadable, ActionChar] = {'stop': '!',
                                                    'inflate': '+',
                                                    'vacuum': '-',
                                                    'release': '^',
                                                    'actuate': 'a'}
    if action in mapping:
        return mapping[action]
    else:
        return action


def _encode_ports(ports):
    return b''.join(reversed([b'1' if p else b'0' for p in ports]))


class FlowIO:
    state: Union[Connected, Literal['not-found', 'disconnected']]
    max_timeout: int = 3

    def __init__(self, comport_path: str):
        self._connect(comport_path)

    @property
    def is_connected(self) -> bool:
        return isinstance(self.state, Connected)

    @property
    def name(self) -> Optional[str]:
        if self.is_connected:
            return self.state.name
        else:
            return None

    @property
    def series(self) -> Optional[str]:
        if self.is_connected:
            return self.state.series
        else:
            return None

    @property
    def role(self) -> Optional[str]:
        if self.is_connected:
            return self.state.role
        else:
            return None

    def _connect(self, comport_path):
        try:
            self.ser = serial.Serial(comport_path, baudrate=230400, timeout=1)
            self.ser.write(b'name?')
            response = self.ser.readline().strip().decode('ascii')
            print(DEBUG_TAG, "response", response)
            if response.startswith('FlowIO_'):
                (series, role) = _parse_name(response)
                self.state = Connected(name=response, series=series, role=role, n_timeout=0)
            else:
                self.ser.close()
                self.state = 'not-found'
        except serial.SerialException as e:
            print(DEBUG_TAG, "exception opening", comport_path, ": ", e)
            if self.ser is not None:
                self.ser.close()

    def command(self,
                action: Union[ActionReadable, ActionChar],
                pwm: int,
                ports: tuple[bool, bool, bool, bool, bool]) -> bool:
        """
        Send a command to connected device
        :param action:
        :param pwm:
        :param ports: 5-tuple representing ports, starting with port 1 (first position) to port 5
        :return: True if command was sent and acknowledge was received. Otherwise False
        """
        if self.is_connected:
            try:
                act = _convert_action(action)
                self.ser.write(b','.join([act.encode('ascii'), str(pwm).encode('ascii'), _encode_ports(ports)]))
                # response = self.ser.readline()
                # if not response.startswith(b'ok'):
                #     # TODO parse the device state
                #     return False
                return True
            except serial.SerialTimeoutException as e:
                print('timeout on device', self.state.name)
                self.state.n_timeout += 1
                if self.state.n_timeout > self.max_timeout:
                    print('disconnecting', self.state.name)
                    self.disconnect()
                return False
        else:
            return False

    def disconnect(self):
        if self.ser is not None:
            self.ser.close()
        self.state = 'disconnected'

    def __str__(self):
        if isinstance(self.state, Connected):
            return '<FlowIO: connected! name=' + self.state.name + '>'
        else:
            return '<FlowIO: ' + self.state + '>'


class Orchestrator:
    schedule: Any

    devices: list[FlowIO]

    def __init__(self):
        self._role_mapping = {'A': '1', 'B': '2', 'C': '3'}
        self.devices = []

    def load_file(self, file):
        with open(file, 'r') as f:
            data = json.load(f)
            self.schedule = data

    def connect_devices(self):
        ports = get_possible_ports()
        raw_devices = [FlowIO(path) for path in ports]
        connected_devices = [d for d in raw_devices if d.is_connected]
        print(DEBUG_TAG, "found", len(connected_devices), "devices:", (d.name for d in connected_devices))
        self.devices = [d for d in connected_devices if d.is_connected and d.role is not None]

    @property
    def device_to_role_mapping(self):
        return self._role_mapping

    @device_to_role_mapping.setter
    def device_to_role_mapping(self, mapping: Mapping[str, str]):
        self._role_mapping = mapping

    async def start(self):
        if self._role_mapping is None:
            print("Missing mappings")
            return
        if len(self.devices) == 0:
            print("not connected devices")
            return
        if self.schedule is None or len(self.schedule) == 0:
            print("no schedule is loaded")
            return
        if isinstance(self.schedule, dict):
            print(DEBUG_TAG, 'converting a single schedule to a sequence')
            self.schedule = [{'partName': 'unknown', 'schedule': self.schedule}]
        for part in self.schedule:
            start_time = time.monotonic()
            print('Playing', part['partName'])
            schedule = part['schedule']
            timing = schedule['time']
            instructions = schedule['instructions']
            roles = instructions.keys()
            devices_for_role = self._arrange_devices_by_roles(roles)

            def elapsed():
                return time.monotonic() - start_time

            def ms_to_sec(ms):
                return ms / 1000

            for i, t in enumerate(timing):
                debug('t =', t, ', i = ', i, ', elapsed =', elapsed())
                while elapsed() < ms_to_sec(t):
                    wait_dur = ms_to_sec(t) - elapsed()
                    debug('waiting', wait_dur, 'sec')
                    await asyncio.sleep(wait_dur)
                for role in roles:
                    instruction = instructions[role][i]
                    for device in devices_for_role[role]:
                        debug('sending', device.name, 'instruction', instruction)
                        device.command(instruction['action'], instruction['pumpPwm'], instruction['ports'])
                debug('done')

    def _arrange_devices_by_roles(self, roles) -> Mapping[str, list[FlowIO]]:
        return {role: [d for d in self.devices if self.device_to_role_mapping[d.role] == role]
                for role in roles}


def get_possible_ports():
    return [('/dev/' + path) for path in listdir('/dev/') if path.startswith('cu.usbmodem')]


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("schedule")
    parser.add_argument('--verbose', '-v', action='store_true')
    args = parser.parse_args()
    DEBUG = args.verbose
    orchestrator = Orchestrator()
    orchestrator.load_file(args.schedule)
    print('Schedule', args.schedule, 'loaded')
    orchestrator.connect_devices()
    print('Devices are connected. Devices:', ','.join(d.name for d in orchestrator.devices))
    print('Device to role mapping is:', orchestrator.device_to_role_mapping)
    asyncio.run(orchestrator.start())
    # paths = get_possible_ports()
    # flowios = [FlowIO(path) for path in paths]
    # connected = [flowio for flowio in flowios if flowio.is_connected()]
    # print('connected:', [str(f) for f in connected])
    # for flowio in connected:
    #     flowio.disconnect()
