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
var_list = []
for var in settings:
    if var["@Accessibility"] == "R/W":
        var_list.append([var["@Item"],var["@Description"]])
print(var_list)


async def get_info(robotip):
    async with techmanpy.connect_svr(robot_ip=robotip) as conn:
        for v in var_list:
            value = await conn.get_value(v[0])
            print(f'{v[1]}: {value}\n')
        
             
        
        
if __name__ == '__main__':
    if len(sys.argv) == 2:
        try:
            asyncio.run(get_info(sys.argv[1]))
        except KeyboardInterrupt:
            print("Error")  # terminate gracefully
    else:
        print(f'usage: {sys.argv[0]} <robot IP address>')