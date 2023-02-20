
import sys
import asyncio
import time
import techmanpy
from techmanpy import TechmanException
import numpy as np

async def test_connection(robot_ip):
    while True:
        start = time.time()
        status = {'SCT': 'offline', 'SVR': 'offline', 'STA': 'offline'}

        # Check SVR connection (should be always active)
        try:
            async with techmanpy.connect_svr(robot_ip=robot_ip, conn_timeout=1) as conn:
                status['SVR'] = 'online'
                await conn.get_value('Robot_Model')
                status['SVR'] = 'connected'
        except TechmanException:
            pass

        # Check SCT connection (only active when inside listen node)
        try:
            async with techmanpy.connect_sct(robot_ip=robot_ip, conn_timeout=1) as conn:
                status['SCT'] = 'online'
                await conn.resume_project()
                status['SCT'] = 'connected'
        except TechmanException:
            pass

        # Check STA connection (only active when running project)
        try:
            async with techmanpy.connect_sta(robot_ip=robot_ip, conn_timeout=1) as conn:
                status['STA'] = 'online'
                await conn.is_listen_node_active()
                status['STA'] = 'connected'
        except TechmanException:
            pass

        # Print status
        def colored(status):
            if status == 'online':
                return f'\033[96m{status}\033[00m'
            if status == 'connected':
                return f'\033[92m{status}\033[00m'
            if status == 'offline':
                return f'\033[91m{status}\033[00m'
        print(
            f'SVR: {colored(status["SVR"])}, SCT: {colored(status["SCT"])}, STA: {colored(status["STA"])}')

        # Sleep 2 seconds (at most)
        elapsed = time.time() - start
        if elapsed < 2:
            time.sleep(2 - elapsed)

async def move(robotip):
    async with techmanpy.connect_sct(robot_ip=robotip) as conn:
        await conn.move_to_joint_angles_ptp([-np.pi, -np.pi/2, np.pi/2, 0, np.pi/2, np.pi], 0.10, 200)

async def move_to_point(robotip):
    async with techmanpy.connect_sct(robot_ip=robotip) as conn:
        await conn.move_to_point_ptp([350, 20, 500, -180, 0, 90], 0.5, 200) # x 不能低於350 z 不能低於95
async def move_to_point2(robotip):
    async with techmanpy.connect_sct(robot_ip=robotip) as conn:
        await conn.move_to_point_ptp([-350, 100, 450, -180, 0, -90], 0.5, 200) # z不能小於350 mm
async def open(robotip):
    async with techmanpy.connect_svr(robot_ip=robotip) as conn:
        await conn.set_value('End_DO0',0)
async def close(robotip):
    async with techmanpy.connect_svr(robot_ip=robotip) as conn:
        await conn.set_value('End_DO0',1)

if __name__ == '__main__':
    if len(sys.argv) == 2:
        try:
            asyncio.run(open(sys.argv[1]))
        except KeyboardInterrupt:
            print("here")  # terminate gracefully
    else:
        print(f'usage: {sys.argv[0]} <robot IP address>')
