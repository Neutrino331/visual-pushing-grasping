import asyncio
import techmanpy
import sys

async def grasp(ip):
    async with techmanpy.connect_svr(robot_ip=ip) as conn:
        await conn.set_value('End_DO0',1)

if __name__ == '__main__':
    if len(sys.argv) == 2:
        try: asyncio.run(grasp(sys.argv[1]))
        except KeyboardInterrupt: print() # terminate gracefully
    else: print(f'usage: {sys.argv[0]} <robot IP address>')