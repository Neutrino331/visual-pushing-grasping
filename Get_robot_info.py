import sys
import asyncio
import time
import techmanpy
from techmanpy import TechmanException
import numpy as np
import xmltodict

xml_file = "./arm_data.xml"
with open(xml_file,encoding="UTF8") as fd:
    data = xmltodict.parse(fd.read())

settings = data["File"]["CodeTable"]["Setting"]
num = 0 
var_list = list()
for var in settings:
    var_list.append(var["@Item"])
#print(var_list)

info = ["Joint_Angle"] # add desired variables

async def get_info(robotip):
    while True:
        async with techmanpy.connect_svr(robot_ip=robotip,client_id="info") as conn:
            value = await conn.get_values(var_list)
            for i in range(len(var_list)):
                print(f'{var_list[i]}:{value[var_list[i]]}')
            # Sleep 2 seconds (at most)
            time.sleep(2)
            await conn.keep_alive()
        
if __name__ == '__main__':
    if len(sys.argv) == 2:
        try:
            asyncio.run(get_info(sys.argv[1]))
        except KeyboardInterrupt:
            print("Error")  # terminate gracefully
    else:
        print(f'usage: {sys.argv[0]} <robot IP address>')