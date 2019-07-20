#!/usr/bin/env python3
# ScratchGPIO - control Raspberry Pi GPIO ports using Scratch.
# Copyright (C) 2013-2019 by Simon Walters based on original code for PiFace by Thomas Preston

# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.

# This code hosted on Github thanks to Ben Nuttall who taught me how to be a git(ter)

Version = 'v0.1_4May19'  # Start
import time
import asyncio, socket

async def handle_client(reader, writer):
    request = None
    while request != 'quit':
        request = (await reader.read(10)).decode('utf8')
        response = str(eval(request)) + '\n'
        writer.write(response.encode('utf8'))

loop = asyncio.get_event_loop()
loop.create_task(asyncio.start_server(handle_client, '192.168.0.189', 42001))
loop.run_forever()