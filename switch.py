import sys
import asyncio
import time
import techmanpy
from techmanpy import TechmanException
import numpy as np

async def open(robotip):
    async with techmanpy.connect_svr(robot_ip=robotip) as conn:
        state = await conn.get_value('End_DO0')
        print(state)
        if state:
            await conn.set_value('End_DO0',0)
        else:
            await conn.set_value('End_DO0',1)

if __name__ == '__main__':
    if len(sys.argv) == 2:
        try:
            asyncio.run(open(sys.argv[1]))
        except KeyboardInterrupt:
            print("Error")  # terminate gracefully
    else:
        print(f'usage: {sys.argv[0]} <robot IP address>')