#!/usr/bin/env python
# coding: utf-8
import transforms3d as tfs
import cv2
import numpy as np
import glob
import asyncio
import math
import serial
import techmanpy
import time

robotip = "192.168.10.13"

chessboard_width_num, chessboard_height_num = (7, 5)

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
objp = np.zeros((5*7, 3), np.float32)
objp[:, :2] = np.mgrid[0:7, 0:5].T.reshape(-1, 2)

hand = []

camera = []
def get_robot_pose():
    async def get_pose(robotip):
        async with techmanpy.connect_svr(robot_ip=robotip, client_id="info") as conn:
            value = await conn.get_values("Coord_Robot_Tool")
            print(f'{"Coord_Robot_Tool"}:{value}')
        return value
    return asyncio.run(get_pose())

def take_picture():
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Cannot open camera")
        exit()
    
    else:
        # 擷取影像
        ret, frame = cap.read()
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
        else:
            cv2.namedWindow("live", cv2.WINDOW_AUTOSIZE); # 命名一個視窗，可不寫
            cv2.imshow("live",frame)
            cv2.imwrite("test_img.png",frame)
            cv2.waitKey(1000)
            return frame
    cv2.destroyAllWindows()

def move(x,y,z,rx,ry,rz):
    async def move_to_point(robotip):
        async with techmanpy.connect_sct(robot_ip=robotip) as conn:
            await conn.move_to_point_ptp([x, y, z, rx, ry, rz], 0.5, 200)
    return asyncio.run(move_to_point())

def get_matrix_eular_radu(x,y,z,rx,ry,rz):
    rmat = tfs.euler.euler2mat(math.radians(rx),math.radians(ry),math.radians(rz))
    rmat = tfs.affines.compose(np.squeeze(np.asarray((x,y,z))), rmat, [1, 1, 1])
    return rmat


def skew(v):
    return np.array([[0,-v[2],v[1]],
                     [v[2],0,-v[0]],
                     [-v[1],v[0],0]])

def rot2quat_minimal(m):
    quat =  tfs.quaternions.mat2quat(m[0:3,0:3])
    return quat[1:]

def quatMinimal2rot(q):
    p = np.dot(q.T,q)
    w = np.sqrt(np.subtract(1,p[0][0]))
    return tfs.quaternions.quat2mat([w,q[0],q[1],q[2]])


axis = np.float32([[0, 0, 0], [0, 3, 0], [3, 3, 0], [3, 0, 0],
                   [0, 0, -3], [0, 3, -3], [3, 3, -3], [3, 0, -3]])


def draw(img, corners, imgpts):
    imgpts = np.int32(imgpts).reshape(-1, 2)
    # draw ground floor in green
    img = cv2.drawContours(img, [imgpts[:4]], -1, (0, 255, 0), -3)
    # draw pillars in blue color
    for i, j in zip(range(4), range(4, 8)):
        img = cv2.line(img, tuple(imgpts[i]), tuple(imgpts[j]), (255), 3)
    # draw top layer in red color
    img = cv2.drawContours(img, [imgpts[4:]], -1, (0, 0, 255), 3)
    return img

for i in range(0,len(hand),6):
    img = take_picture()
    gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    print("gray_img shape:",gray_img.shape)

    cp_int = np.zeros((chessboard_width_num * chessboard_height_num, 3), np.float32)
    cp_int[:, :2] = np.mgrid[0:chessboard_width_num, 0:chessboard_height_num].T.reshape(-1, 2)
    # cp_world: corner point in world space, save the coordinate of corner points in world space.
    cp_world = cp_int * 0.02

    ret2, corners = cv2.findChessboardCorners(gray_img, (chessboard_width_num, chessboard_height_num), None)
    obj_points = []  # the points in world space
    img_points = []  # the points in image space (relevant to obj_points)
    obj_points.append(cp_world)
    img_points.append(corners)

    if ret2 == True:
        # view the corners
        cv2.drawChessboardCorners(img, (chessboard_width_num, chessboard_height_num), corners, ret2)
        _, mtx, dist, v_rot, v_trans = cv2.calibrateCamera(
            obj_points, img_points, gray_img.shape[::-1], None, None)
        corners2 = cv2.cornerSubPix(
            gray_img, corners, (11, 11), (-1, -1), criteria)
        # Find the rotation and translation vectors.
        _, rvecs, tvecs = cv2.solvePnP(objp, corners2, mtx, dist)
        # project 3D points to image plane
        imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, mtx, dist)
        img = draw(img, corners2, imgpts)
    else:
        print("Can't find chessboard!")

    cv2.imshow('img', img)
    print("内部参數=", mtx)
    print("扭曲系数=", dist)
    print(f"第{i}組旋轉向量=", v_rot)
    print(f"第{i}組平移向量=", v_trans)
    print("="*25)
    camera.append([v_trans[0], v_trans[1], v_trans[2],
                  v_rot[0], v_rot[1], v_rot[2]])

Hgs, Hcs = [], []

for i in range(0,len(hand),6):
    Hgs.append(get_matrix_eular_radu(hand[i],hand[i+1],hand[i+2],hand[i+3],hand[i+4],hand[i+5]))    
    Hcs.append(get_matrix_eular_radu(camera[i],camera[i+1],camera[i+2],camera[i+3],camera[i+4],camera[i+5]))

Hgijs = []
Hcijs = []
A = []
B = []
size = 0
for i in range(len(Hgs)):
    for j in range(i+1, len(Hgs)):
        size += 1
        Hgij = np.dot(np.linalg.inv(Hgs[j]), Hgs[i])
        Hgijs.append(Hgij)
        Pgij = np.dot(2, rot2quat_minimal(Hgij))

        Hcij = np.dot(Hcs[j], np.linalg.inv(Hcs[i]))
        Hcijs.append(Hcij)
        Pcij = np.dot(2, rot2quat_minimal(Hcij))

        A.append(skew(np.add(Pgij, Pcij)))
        B.append(np.subtract(Pcij, Pgij))
MA = np.asarray(A).reshape(size*3, 3)
MB = np.asarray(B).reshape(size*3, 1)
Pcg_ = np.dot(np.linalg.pinv(MA), MB)
pcg_norm = np.dot(np.conjugate(Pcg_).T, Pcg_)
Pcg = np.sqrt(np.add(1, np.dot(Pcg_.T, Pcg_)))
Pcg = np.dot(np.dot(2, Pcg_), np.linalg.inv(Pcg))
Rcg = quatMinimal2rot(np.divide(Pcg, 2)).reshape(3, 3)

A = []
B = []
id = 0
for i in range(len(Hgs)):
    for j in range(i+1, len(Hgs)):
        Hgij = Hgijs[id]
        Hcij = Hcijs[id]
        A.append(np.subtract(Hgij[0:3, 0:3], np.eye(3, 3)))
        B.append(np.subtract(np.dot(Rcg, Hcij[0:3, 3:4]), Hgij[0:3, 3:4]))
        id += 1

MA = np.asarray(A).reshape(size*3, 3)
MB = np.asarray(B).reshape(size*3, 1)
Tcg = np.dot(np.linalg.pinv(MA), MB).reshape(3,)
print(tfs.affines.compose(Tcg, np.squeeze(Rcg), [1, 1, 1]))
