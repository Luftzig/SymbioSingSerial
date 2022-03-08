from collections import namedtuple
from typing import Literal, Mapping, Union, Optional

import serial

from symbiosing.utils import DEBUG_TAG, debug

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
            self.ser.write(b'name?\n')
            self.ser.flush()
            response = self.ser.readline()
            debug("response", response)
            response = response.strip().decode('ascii')
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
                self.ser.write(b','.join([act.encode('ascii'), str(pwm).encode('ascii'), _encode_ports(ports)]) + b'\n')
                # self.ser.flush()
                # response = self.ser.readline()
                # debug("command response is:", response)
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