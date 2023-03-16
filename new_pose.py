import numpy as np
import cv2
import pyrealsense2 as rs
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30) 

cfg = pipeline.start(config)
profile = cfg.get_stream(rs.stream.color)

intr = profile.as_video_stream_profile().get_intrinsics()
print(intr)
def draw(img, corners, imgpts):
    corner = tuple(corners[0].ravel())
    
    img = cv2.line(img, int(corner), tuple(imgpts[0].ravel()), (255,0,0), 5)
    img = cv2.line(img, int(corner), tuple(imgpts[1].ravel()), (0,255,0), 5)
    img = cv2.line(img, int(corner), tuple(imgpts[2].ravel()), (0,0,255), 5)
    return img
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
objp = np.zeros((5*7,3), np.float32)
objp[:,:2] = np.mgrid[0:7,0:5].T.reshape(-1,2)
axis = np.float32([[3,0,0], [0,3,0], [0,0,-3]]).reshape(-1,3)
objpoints = []
imgpoints = []
while(True):
    frames = pipeline.wait_for_frames()  #等待获取图像帧
    color_frame = frames.get_color_frame()
    color_image = np.asarray(color_frame.get_data())  # RGB图
    cv2.imshow("img",color_image)
    gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, (7,5), None)
    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)
        corners2 = cv2.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners2)
        # Draw and display the corners
        cv2.drawChessboardCorners(gray, (7,5), corners2, ret)
        
        _, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
        
        # Find the rotation and translation vectors.
        ret,rvecs, tvecs = cv2.solvePnP(objp, corners2, mtx, dist)
        # project 3D points to image plane
        imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, mtx, dist)
        print(corners2)
        img = draw(gray,corners2,imgpts)
        cv2.imshow('img',gray)
        k = cv2.waitKey(0) & 0xFF
        if k == ord('q'):
            break
cv2.destroyAllWindows()