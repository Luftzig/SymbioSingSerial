# Based on Fehmi's project
from bleak import BleakScanner, BleakClient, BleakError
from bleak.exc import BleakError
from struct import pack
import asyncio
import json
import pickle
from multiprocessing.connection import wait
import traceback
import asyncio
import socketio
import time
from datetime import datetime
# import socket programming library
import socket

indicator_chr = '0b0b0b0b-0b0b-0b0b-0b0b-c1000000aa02'
indicator_err_chr = '0b0b0b0b-0b0b-0b0b-0b0b-c2000000aa02'
command_chr = '0b0b0b0b-0b0b-0b0b-0b0b-c1000000aa04'
hardware_chr = '0b0b0b0b-0b0b-0b0b-0b0b-c2000000aa04'
config_chr = '0b0b0b0b-0b0b-0b0b-0b0b-c1000000aa03'

COMMANDS = {
    'stop_all': b'\x21\x1F',
    'release': b'\x5e',
    'inflate': b'\x2b',
    'vacuum': b'\x2d',
    'open': b'\x6f',
    'inflation_series': b'\x01',
    'stop': b'\x21'
}


class FlowIO():
    def __init__(self, ble_names, conn_gui, conn_sync, DEBUGGING=False):
        self.ble_names = ble_names
        self.conn_sync = conn_sync
        self.conn_gui = conn_gui
        self.debugging = DEBUGGING
        self.exit = False
        self.schedule = None

        loop = asyncio.get_event_loop()
        task = self.discover()
        self.devices = loop.run_until_complete(asyncio.gather(task))[0]
        if len(self.devices) > 0:
            # We found devices
            returned = loop.run_until_complete(asyncio.gather(self.connect()))[0]
            if returned:
                # We established connection with audio and gui
                self.clients = returned
            else:
                print('Unable to establish connection with the devices')
                self.exit = True
        else:
            self.exit = True

    async def discover(self):
        print('Turn the FlowIO boxes on. Looking for', ', '.join(self.ble_names))

        # Scanning for devices
        discovered_devices = await BleakScanner.discover()
        devices = dict()
        for d in discovered_devices:
            found_name = d.name
            if 'FlowIO_' in found_name:
                found_name = found_name.split('_')[1]
            if self.debugging:
                if found_name == "CtrPwrCfgIndBatPrs":
                    found_name = '1A'
            if found_name in self.ble_names:
                devices[found_name] = d.address

        # Let's see what we have
        if len(devices) == 0:
            print('No device found. Turn the FlowIO boxes off and on again, and check that they are in range.')
            return devices
        else:
            print('Found', ', '.join(sorted(list(devices.keys()))))

        return devices

    async def connect(self):

        # Connecting all devices found
        clients = dict()
        for device in self.devices:
            clients[device] = BleakClient(self.devices[device])
            try:
                await clients[device].connect()
            except:
                print(device, 'not connected')

        return clients

    async def disconnect(self):
        for client in self.clients:
            await self.clients[client].disconnect()

    def listening(self):
        # Listening routine
        try:
            self.queue = []
            while True:
                try:
                    conn_list = wait([self.conn_gui, self.conn_sync], timeout=-1)
                    # if we got a message, empty the buffer to the queue and check there is no urgent message
                    if len(conn_list) > 0:
                        for conn in conn_list:
                            data = conn.recv()
                            data_dic = pickle.loads(data, encoding='bytes')
                            # if audio requests silence, we remove all short schedules
                            if data_dic['command'] == 'silence':
                                self.queue = [item for item in self.queue if item['command'] != 'short']
                            if data_dic['command'] == 'stop':
                                self.pprint('Stop command received.')
                                self.queue = []
                            # otherwise we add it to the queue
                            else:
                                self.queue.append(data_dic)
                    # otherwise keep emptying the queue
                    elif len(self.queue) > 0:
                        data_dic = self.queue.pop(0)
                        if data_dic['command'] == 'short':
                            # self.test()
                            pass
                        if data_dic['command'] == 'play':
                            self.load(data_dic['data'])
                        if data_dic['command'] == 'start':
                            self.start()
                except (EOFError, ConnectionResetError):
                    # traceback.print_exc()
                    break
        except Exception:
            traceback.print_exc()
            pass

        print('FlowIO listener quitting.')
        return False

    async def toggle_led(self, client):
        indicator_val = await client.read_gatt_char(indicator_chr)
        if indicator_val == bytearray(b'\x00\x01'):
            await client.write_gatt_char(indicator_chr, b'\x01\x01')
        else:
            await client.write_gatt_char(indicator_chr, bytearray(b'\x00\x01'))

    async def config(self, client, action):
        await client.write_gatt_char(config_chr, action)

    async def command(self, client, action, delay, ports):
        #if action == COMMANDS['inflate']:
        #    await self.toggle_led(client)
        # print(datetime.datetime.now(), 'sending', client, 'action:', action, 'ports', ports)
        start_time = time.time()
        await client.write_gatt_char(command_chr, action + pack('b', int(ports, 2)))
        sent_time = time.time()
        await asyncio.sleep((delay / 1000) - (sent_time - start_time))

    def load(self, file):
        json_file = open(file, 'r')
        parts = json.load(json_file)
        self.schedule = parts
        self.pprint('Loaded: ' + str(file.split('/')[-1]))

    def start(self):
        if self.schedule == None:
            self.pprint('No schedule loaded.')
            return False
        parts = self.schedule
        stop = False
        for part in parts:
            if stop:
                break
            self.pprint(10 * '_')
            self.pprint('Part ' + part['partName'])
            seq = part['schedule']
            times = seq['time']
            #### SEQUENCE STARTS HERE ####
            loop = asyncio.get_event_loop()
            for count in range(len(times) - 1):
                conn_list = wait([self.conn_gui], timeout=-1)
                # if we got a message, empty the buffer to the queue and check there is no urgent message
                if len(conn_list) > 0:
                    conn = conn_list[0]
                    data = conn.recv()
                    data_dic = pickle.loads(data, encoding='bytes')
                    if data_dic['command'] == 'stop':
                        self.pprint('Schedule aborted by user.')
                        stop = True
                        break
                    else:
                        self.queue.append(data_dic)
                tasks = [self.play_schedule(device, seq, count, times) for device in self.clients]
                loop.run_until_complete(asyncio.gather(*tasks))
        self.pprint('End of schedule.')
        ##############################

    async def play_schedule(self, device, seq, count, times):
        roles = {'A': '1',
                 'B': '2',
                 'C': '3'}
        try:
            role = roles[device[1]]
            ports_list = seq['instructions'][role][count]['ports']
            ports_binary = ''.join(['1' if val else '0' for val in ports_list])
            com = seq['instructions'][role][count]['action']
            if com == 'actuate':
                com = 'inflate'
            try:
                delay = times[count + 1] - times[count]
            except IndexError:
                delay = 0
            print(datetime.now().strftime("%H:%M:%S") + "\t" +
                  str(count) + '\t' + str(device) + '\t' + str(times[count]).rjust(5) + '\t' +
                         str(delay).rjust(5) + '\t' + com.ljust(7) + '\t' + str(ports_binary))
            await self.command(self.clients[device], COMMANDS[com], delay, ports_binary)
        except BleakError:
            self.pprint(5 * '!' + ' ' + device + ' disconnected ' + 5 * '!')
        except KeyError:
            print('Failed to get command for', device, count)
        except IndexError:
            print('Failed to get command for', device, count)

    def test(self):
        loop = asyncio.get_event_loop()
        try:
            tasks = [self.command(self.clients[device], COMMANDS['inflate'], 100, '11111') for device in self.clients]
            loop.run_until_complete(asyncio.gather(*tasks))
        except BleakError:
            print('FlowIO disconnected. Check that it is turned on and in range.')
            # traceback.print_exc()
            return False
        return True

    def pprint(self, text):
        serialised_gui = pickle.dumps({
            'command': 'print',
            'data': text
        })
        try:
            self.conn_gui.send(serialised_gui)
        except:
            print("Couldn't send to GUI process.")
            self.pause = True


"""
INFO:__main__:[Service] 0b0b0b0b-0b0b-0b0b-0b0b-00000000aa04 (Handle: 14): Unknown
INFO:__main__:	[Characteristic] 0b0b0b0b-0b0b-0b0b-0b0b-c1000000aa04 (Handle: 15): Unknown (write), Value: None
INFO:__main__:	[Characteristic] 0b0b0b0b-0b0b-0b0b-0b0b-c2000000aa04 (Handle: 17): Unknown (read,notify), Value: b'\x00\x00'
INFO:__main__:		[Descriptor] 00002902-0000-1000-8000-00805f9b34fb (Handle: 19): Client Characteristic Configuration) | Value: b'\x00\x00'
INFO:__main__:[Service] 0b0b0b0b-0b0b-0b0b-0b0b-00000000aa01 (Handle: 20): Unknown
INFO:__main__:	[Characteristic] 0b0b0b0b-0b0b-0b0b-0b0b-c1000000aa01 (Handle: 21): Unknown (read,write,notify), Value: b'\x05'
INFO:__main__:		[Descriptor] 00002902-0000-1000-8000-00805f9b34fb (Handle: 23): Client Characteristic Configuration) | Value: b'\x00\x00'
configService:[Service] 0b0b0b0b-0b0b-0b0b-0b0b-00000000aa03 (Handle: 24): Unknown
chrConfig    :	[Characteristic] 0b0b0b0b-0b0b-0b0b-0b0b-c1000000aa03 (Handle: 25): Unknown (read,write), Value: b'\x00'
INFO:__main__:[Service] 0b0b0b0b-0b0b-0b0b-0b0b-00000000aa02 (Handle: 27): Unknown
indicator_chr:	[Characteristic] 0b0b0b0b-0b0b-0b0b-0b0b-c1000000aa02 (Handle: 28): Unknown (read,write,notify), Value: b'\x00\x01'
INFO:__main__:		[Descriptor] 00002902-0000-1000-8000-00805f9b34fb (Handle: 30): Client Characteristic Configuration) | Value: b'\x00\x00'
INFO:__main__:	[Characteristic] 0b0b0b0b-0b0b-0b0b-0b0b-c2000000aa02 (Handle: 31): Unknown (read,write,notify), Value: b'\x00'
INFO:__main__:		[Descriptor] 00002902-0000-1000-8000-00805f9b34fb (Handle: 33): Client Characteristic Configuration) | Value: b'\x00\x00'
INFO:__main__:[Service] 0000180f-0000-1000-8000-00805f9b34fb (Handle: 34): Battery Service
INFO:__main__:	[Characteristic] 00002a19-0000-1000-8000-00805f9b34fb (Handle: 35): Battery Level (read,notify), Value: b'f'
INFO:__main__:		[Descriptor] 00002902-0000-1000-8000-00805f9b34fb (Handle: 37): Client Characteristic Configuration) | Value: b'\x00\x00'
INFO:__main__:[Service] 0b0b0b0b-0b0b-0b0b-0b0b-00000000aa05 (Handle: 38): Unknown
INFO:__main__:	[Characteristic] 00002a6d-0000-1000-8000-00805f9b34fb (Handle: 39): Pressure (read,write,notify), Value: b'`"fA'
INFO:__main__:		[Descriptor] 00002902-0000-1000-8000-00805f9b34fb (Handle: 41): Client Characteristic Configuration) | Value: b'\x00\x00'
INFO:__main__:	[Characteristic] 0b0b0b0b-0b0b-0b0b-0b0b-c3000000aa05 (Handle: 42): Unknown (read,write,notify), Value: b'\xfeZ\xa7\xf7\x97\xfb}\xb3[\x1f|\xf2\x1d\xf6\xff(\x1bx\xc4.'
INFO:__main__:		[Descriptor] 00002902-0000-1000-8000-00805f9b34fb (Handle: 44): Client Characteristic Configuration) | Value: b'\x00\x00'
INFO:__main__:	[Characteristic] 0b0b0b0b-0b0b-0b0b-0b0b-c4000000aa05 (Handle: 45): Unknown (read,write,notify), Value: b'\x8d\x8c\xfdc\xc8\xdb+3Q\x02"\xfa\x83+a\x08\xe7\x9d\xe6\xe9'
INFO:__main__:		[Descriptor] 00002902-0000-1000-8000-00805f9b34fb (Handle: 47): Client Characteristic Configuration) | Value: b'\x00\x00'
"""
