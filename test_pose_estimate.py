import numpy as np
import cv2 as cv
import glob
import time 
import pyrealsense2 as rs

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30) 

cfg = pipeline.start(config)
profile = cfg.get_stream(rs.stream.color)

intr = profile.as_video_stream_profile().get_intrinsics()
c = 0
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
objp = np.zeros((5*7, 3), np.float32)
objp[:, :2] = np.mgrid[0:7, 0:5].T.reshape(-1, 2)

axis = np.float32([[0, 0, 0], [0, 3, 0], [3, 3, 0], [3, 0, 0],
                   [0, 0, -3], [0, 3, -3], [3, 3, -3], [3, 0, -3]])

def draw(img, corners, imgpts):
    imgpts = np.int32(imgpts).reshape(-1, 2)
    # draw ground floor in green
    img = cv.drawContours(img, [imgpts[:4]], -1, (0, 255, 0), -3)
    # draw pillars in blue color
    for i, j in zip(range(4), range(4, 8)):
        img = cv.line(img, tuple(imgpts[i]), tuple(imgpts[j]), (255), 3)
    # draw top layer in red color
    img = cv.drawContours(img, [imgpts[4:]], -1, (0, 0, 255), 3)
    return img

cap = cv.VideoCapture(0)
if not cap.isOpened():
    print("Cannot open camera")
    exit()

#cv2.namedWindow("live", cv2.WINDOW_AUTOSIZE); # 命名一個視窗，可不寫
while(True):
    # 擷取影像
    frames = pipeline.wait_for_frames()  #等待获取图像帧
    color_frame = frames.get_color_frame()
    color_image = np.asarray(color_frame.get_data())  # RGB图

    gray_img = cv.cvtColor(color_image, cv.COLOR_BGR2GRAY)
    print(gray_img.shape)
    w1, h1 = (7, 5)
    # w2,h2=(11,8)

    cp_int = np.zeros((w1 * h1, 3), np.float32)
    cp_int[:, :2] = np.mgrid[0:w1, 0:h1].T.reshape(-1, 2)
    # cp_world: corner point in world space, save the coordinate of corner points in world space.
    cp_world = cp_int * 0.02

    ret2, corners = cv.findChessboardCorners(gray_img, (w1, h1), None)
    #print(ret, corners)
    obj_points = []  # the points in world space
    img_points = []  # the points in image space (relevant to obj_points)
    obj_points.append(cp_world)
    img_points.append(corners)
    
    if ret2 == True:
        # view the corners
        cv.drawChessboardCorners(color_image, (w1, h1), corners, ret2)
        _, mtx, dist, v_rot, v_trans = cv.calibrateCamera(
            obj_points, img_points, gray_img.shape[::-1], None, None)
        corners2 = cv.cornerSubPix(gray_img, corners, (11, 11), (-1, -1), criteria)
        # Find the rotation and translation vectors.
        _, rvecs, tvecs = cv.solvePnP(objp, corners2, mtx, dist)
        # project 3D points to image plane
        imgpts, jac = cv.projectPoints(axis, rvecs, tvecs, mtx, dist)
        frame = draw(color_image, corners2, imgpts)
    cv.imshow('img', color_image)
    cv.waitKey(1000)
    print("内参=", mtx)
    print("畸变系数=", dist)
    print("旋转向量=", v_rot)
    print("平移向量=", v_trans)
    print("="*50)
    k = cv.waitKey(67)
    if k == ord('s'):
        c = c+1
        cv.imwrite('./cool'+str(c)+'.png', color_image)
    elif k== ord('q'):
        break
cv.destroyAllWindows()
with open("./intrinsics.txt", 'w',encoding="UTF8") as f:
    f.write(f"内参={mtx}\n")
    f.write(f'畸变系数={dist}\n')
    f.write(f"旋转向量={v_rot}\n")
    f.write(f"平移向量={v_trans}\n")
    

