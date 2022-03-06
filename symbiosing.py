import argparse

import asyncio

from symbiosing import gui, utils
from symbiosing.orchestrator import Orchestrator


async def cli_only():
    orchestrator = Orchestrator()
    orchestrator.load_file(args.schedule)
    print('Schedule', args.schedule, 'loaded')
    orchestrator.connect_devices()
    print('Devices are connected. Devices:', ','.join(d.name for d in orchestrator.devices))
    print('Device to role mapping is:', orchestrator.device_to_role_mapping)
    await orchestrator.start()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--schedule", "-S")
    parser.add_argument('--verbose', '-v', action='store_true')
    args = parser.parse_args()
    utils.DEBUG = args.verbose
    if args.schedule:
        asyncio.run(cli_only())
    else:
        asyncio.run(gui.show())
    # paths = get_possible_ports()
    # flowios = [FlowIO(path) for path in paths]
    # connected = [flowio for flowio in flowios if flowio.is_connected()]
    # print('connected:', [str(f) for f in connected])
    # for flowio in connected:
    #     flowio.disconnect()
