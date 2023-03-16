import cv2
import numpy as np
import glob
from math import *
import pandas as pd
import os
import pyrealsense2 as rs
import asyncio
import time
import techmanpy
from techmanpy import TechmanException

def get_data():
    pipeline = rs.pipeline()  #定义流程pipeline
    config = rs.config()   #定义配置config
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)  #配置depth流
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)   #配置color流
    profile = pipeline.start(config)  #流程开始
    align_to = rs.stream.color  #与color流对齐
    align = rs.align(align_to)
    frames = pipeline.wait_for_frames()  #等待获取图像帧
    aligned_frames = align.process(frames)  #获取对齐帧
    aligned_depth_frame = aligned_frames.get_depth_frame()  #获取对齐帧中的depth帧
    color_frame = aligned_frames.get_color_frame()   #获取对齐帧中的color帧
    intr = color_frame.profile.as_video_stream_profile().intrinsics   #获取相机内参
    depth_intrin = aligned_depth_frame.profile.as_video_stream_profile().intrinsics  #获取深度参数（像素坐标系转相机坐标系会用到）
    depth_image = np.asanyarray(aligned_depth_frame.get_data())  #深度图（默认16位）
    depth_image_8bit = cv2.convertScaleAbs(depth_image, alpha=0.03)  #深度图（8位）
    color_image = np.asanyarray(color_frame.get_data())  # RGB图
    intrinsics = np.asarray( [[intr.fx, 0, intr.ppx], [0,  intr.fy, intr.ppy], [0, 0, 1]])
    return intrinsics

chess_board_x_num=7#棋盘格x方向格子数
chess_board_y_num=5#棋盘格y方向格子数
chess_board_len=30#单位棋盘格长度,mm

def move_to(p, p1, p2, o1, o2, o3):
    async def move_to_point(p,p1,p2,o1,o2,o3):
        async with techmanpy.connect_sct(robot_ip='192.168.10.13') as conn:
            await conn.move_to_point_ptp([p, p1, p2, o1, o2, o3], 0.25, 200)
    asyncio.run(move_to_point(p, p1, p2, o1, o2, o3))

    #用于根据欧拉角计算旋转矩阵
def myRPY2R_robot(x, y, z):
    Rx = np.array([[1, 0, 0], [0, cos(x), -sin(x)], [0, sin(x), cos(x)]])
    Ry = np.array([[cos(y), 0, sin(y)], [0, 1, 0], [-sin(y), 0, cos(y)]])
    Rz = np.array([[cos(z), -sin(z), 0], [sin(z), cos(z), 0], [0, 0, 1]])
    R = Rz@Ry@Rx
    return R

#用于根据位姿计算变换矩阵
def pose_robot(x, y, z, Tx, Ty, Tz):
    thetaX = x / 180 * pi
    thetaY = y / 180 * pi
    thetaZ = z / 180 * pi
    R = myRPY2R_robot(thetaX, thetaY, thetaZ)
    t = np.array([[Tx], [Ty], [Tz]])
    RT1 = np.column_stack([R, t])  # 列合并
    RT1 = np.row_stack((RT1, np.array([0,0,0,1])))
    # RT1=np.linalg.inv(RT1)
    return RT1

#用来从棋盘格图片得到相机外参
def get_RT_from_chessboard(i, pose, chess_board_x_num,chess_board_y_num,chess_board_len):
   
    move_to(pose[i][0],pose[i][1],pose[i][2],pose[i][3],pose[i][4],pose[i][5])
    print(pose[i][0],pose[i][1],pose[i][2],pose[i][3],pose[i][4],pose[i][5])
    time.sleep(3)
    pipeline = rs.pipeline()  #定义流程pipeline
    config = rs.config()   #定义配置config
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)  #配置depth流
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)   #配置color流
    profile = pipeline.start(config)  #流程开始
    align_to = rs.stream.color  #与color流对齐
    align = rs.align(align_to)
    for i in range(0,30):
        frames = pipeline.wait_for_frames()  #等待获取图像帧
        aligned_frames = align.process(frames)  #获取对齐帧
        aligned_depth_frame = aligned_frames.get_depth_frame()  #获取对齐帧中的depth帧
        color_frame = aligned_frames.get_color_frame()   #获取对齐帧中的color帧
        intr = color_frame.profile.as_video_stream_profile().intrinsics   #获取相机内参

        depth_intrin = aligned_depth_frame.profile.as_video_stream_profile().intrinsics  #获取深度参数（像素坐标系转相机坐标系会用到）
        color_image = np.asanyarray(color_frame.get_data())  # RGB图
    print(intr)
    gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
    cv2.imshow("img", gray)
    cv2.waitKey(2)
    size = gray.shape[::-1]
    ret, corners = cv2.findChessboardCorners(gray, (chess_board_x_num, chess_board_y_num), None)
    # print(corners)
    corner_points=np.zeros((2,corners.shape[0]),dtype=np.float64)
    for i in range(corners.shape[0]):
        corner_points[:,i]=corners[i,0,:]
    # print(corner_points)
    object_points=np.zeros((3,chess_board_x_num*chess_board_y_num),dtype=np.float64)
    flag=0
    for i in range(chess_board_y_num):
        for j in range(chess_board_x_num):
            object_points[:2,flag]=np.array([(7-j-1)*chess_board_len,(5-i-1)*chess_board_len])
            flag+=1
    # print(object_points)
    intrinsics = get_data()
    retval,rvec,tvec  = cv2.solvePnP(object_points.T,corner_points.T, intrinsics, distCoeffs=None)
    # print(rvec.reshape((1,3)))
    # RT=np.column_stack((rvec,tvec))
    RT=np.column_stack(((cv2.Rodrigues(rvec))[0],tvec))
    """ print(rvec)
    def rot_params_rv(rvecs):
        from math import pi,atan2,asin
        R = cv2.Rodrigues(rvecs)[0]
        print(R)
        roll = 180*atan2(-R[2][1], R[2][2])/pi
        pitch = 180*asin(R[2][0])/pi
        yaw = 180*atan2(-R[1][0], R[0][0])/pi
        rot_params= [roll,pitch,yaw]
        return rot_params
    print(rot_params_rv(rvec)) """
    RT = np.row_stack((RT, np.array([0, 0, 0, 1])))
    # RT=pose(rvec[0,0],rvec[1,0],rvec[2,0],tvec[0,0],tvec[1,0],tvec[2,0])
    # print(RT)
    with open("./intrinsics.txt", 'w',encoding="UTF8") as f:
        f.write(f"内参=\n{intrinsics}\n")
        
    # print(retval, rvec, tvec)
    # print(RT)
    # print('')
    return RT

pose = [[98.48088, -580.5756, 137.185, 166.851, 5.214443, -8.11066],
[86.70583, -580.8532, 213.8269, 177.3208, -31.93295, -27.82692],
[218.788, -513.8047, 178.1355, 147.2267, -0.3625326, -44.94373],
[88.23839, -601.2761, 52.41852, 176.3424, -7.565168, -6.573181],
[192.8963, -518.6313, 197.3694, 141.8427, 16.6704, -27.7069],
[-6.172165, -565.1449, 111.1683, 148.3893, -21.0301, 41.41455],
[206.7135, -596.6494, 156.2145, 163.9829, -6.41746, -14.29395],
[127.5931, -523.8399, 108.8214, 137.9044, -3.270428, 0.4479634],
[160.6458, -491.9189, 112.2409, 142.6728, 19.38161, 10.55346],
[50.34186, -554.9487, 82.25433, 141.5474, -0.8874476, 24.1776]]

#计算board to cam 变换矩阵
R_all_chess_to_cam_1=[]
T_all_chess_to_cam_1=[]
for i in range(0, len(pose)):
    RT=get_RT_from_chessboard(i, pose, chess_board_x_num, chess_board_y_num, chess_board_len)
    # RT=np.linalg.inv(RT)
    R_all_chess_to_cam_1.append(RT[:3,:3])
    T_all_chess_to_cam_1.append(RT[:3, 3].reshape((3,1)))
# print(T_all_chess_to_cam.shape)

#计算end to base变换矩阵
R_all_end_to_base_1=[]
T_all_end_to_base_1=[]
# print(sheet_1.iloc[0]['ax'])
for i in range(0,len(pose)):
    # print(sheet_1.iloc[i-1]['ax'],sheet_1.iloc[i-1]['ay'],sheet_1.iloc[i-1]['az'],sheet_1.iloc[i-1]['dx'],
    #           
    #                         sheet_1.iloc[i-1]['dy'],sheet_1.iloc[i-1]['dz'])
   
    RT=pose_robot(pose[i][3],pose[i][4],pose[i][5], pose[i][0],pose[i][1],pose[i][2])
    # RT=np.column_stack(((cv2.Rodrigues(np.array([[sheet_1.iloc[i-1]['ax']],[sheet_1.iloc[i-1]['ay']],[sheet_1.iloc[i-1]['az']]])))[0],
    #                    np.array([[sheet_1.iloc[i-1]['dx']],
    #                                   [sheet_1.iloc[i-1]['dy']],[sheet_1.iloc[i-1]['dz']]])))
    # RT = np.row_stack((RT, np.array([0, 0, 0, 1])))
    # RT = np.linalg.inv(RT)

    R_all_end_to_base_1.append(RT[:3, :3])
    T_all_end_to_base_1.append(RT[:3, 3].reshape((3, 1)))
    # print(R_all_end_to_base_1[i])

# print(R_all_end_to_base_1)
R,T=cv2.calibrateHandEye(R_all_end_to_base_1,T_all_end_to_base_1,R_all_chess_to_cam_1,T_all_chess_to_cam_1,cv2.CALIB_HAND_EYE_TSAI)#手眼标定
RT=np.column_stack((R,T))
RT = np.row_stack((RT, np.array([0, 0, 0, 1])))#即为cam to end变换矩阵
print('相機對於末端的轉換矩陣：')
print(RT)
with open("./intrinsics.txt", 'a',encoding="UTF8") as f:
        f.write(f"相機對於末端的轉換矩陣：\n{RT}")

#结果验证，原则上来说，每次结果相差较小
for i in range(0,len(pose)):

    RT_end_to_base=np.column_stack((R_all_end_to_base_1[i],T_all_end_to_base_1[i]))
    RT_end_to_base=np.row_stack((RT_end_to_base,np.array([0,0,0,1])))
    # print(RT_end_to_base)

    RT_chess_to_cam=np.column_stack((R_all_chess_to_cam_1[i],T_all_chess_to_cam_1[i]))
    RT_chess_to_cam=np.row_stack((RT_chess_to_cam,np.array([0,0,0,1])))
    # print(RT_chess_to_cam)

    RT_cam_to_end=np.column_stack((R,T))
    RT_cam_to_end=np.row_stack((RT_cam_to_end,np.array([0,0,0,1])))
    # print(RT_cam_to_end)
    
    RT_chess_to_base=RT_end_to_base@RT_cam_to_end@RT_chess_to_cam #即为固定的棋盘格相对于机器人基坐标系位姿
    RT_chess_to_base=np.linalg.inv(RT_chess_to_base)
    camtobase =RT_end_to_base@RT_cam_to_end
    print('第',i,'次')
    print(camtobase[:3,:])
    print('')
