import json
import time
from os import listdir
from typing import Any, Mapping

import asyncio

from symbiosing.flowio import FlowIO
from symbiosing.utils import DEBUG_TAG, debug


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
        debug("found", len(connected_devices), "devices:", list(d.name for d in connected_devices))
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
            print("No connected FlowIO devices")
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