import cv2
import numpy as np
import glob
import math
import serial
import techmanpy
import time

serialHandle = serial.Serial("/dev/ttyAMA0", 115200)  #初始化串口， 波特率为115200

command = {"MOVE_WRITE":1, "POS_READ":28, "LOAD_UNLOAD_WRITE": 31}

gripperPose = np.array([[ -47.46614798,-30.93409332,-106.33248484,-2.66670287,0.17899222,1.52106046],
                        [ -50.64555014,-25.69099839,-106.03448307,-2.60620767,0.31446276,1.49742558],
                        [-4.11761269e+01,-3.96028022e+01,-1.05198809e+02,2.69146962e+00,8.27020253e-02,-1.55768099e+00],
                        [ -35.24396468,-44.69683285,-105.73538777,2.62497237,0.25738079,-1.51919588],
                        [-1.35740504e+02,-5.24577701e+01,-1.42120520e+02,2.49769933e+00,1.39643072e-01,-1.79081589e+00],
                        [-1.40319905e+02,-3.85697620e+01,-1.42120520e+02,2.54783819e+00,1.42299609e-02,-1.82676476e+00],
                        [-1.40607129e+02,-3.85729704e+01,-1.41216360e+02,2.54206370e+00,1.41977097e-02,-1.83474341e+00],
                        [-144.89888667,-16.20343465,-141.21636039,-2.4742213,0.1799589,1.78577792],
                        [-2.87355186e+01,-3.54512169e+01,-1.17795839e+02,-2.94734829e+00,8.23268923e-02,.02983728e+00],
                        [ -23.97006612,-40.2412104,-119.7291674,2.85844706,0.17583165,-1.17350036],
                        [ -34.66609758,-31.49890098,-119.7291674,-2.83632764,0.24615135,1.16441951],
                        [ -33.0391174,-33.69010423,-118.9732689,-2.86150817,0.15196856,1.18527549],
                        [-5.87034928e+01,-3.25139467e+01,-1.52581512e+02,-2.84878652e+00,1.11431156e-01,1.24342019e+00],
                        [-1.20653741e+01,-3.61680661e+01,-1.65803499e+02,-3.04942726e+00,1.19279280e-01,6.71576060e-01],
                        [-7.21552695e+01,-4.10918877e+01,-2.20040184e+02,3.12903017e+00,8.74017266e-02,-2.16608054e-01],
                        [-2.71427089e+01,-3.14464549e+01,-2.10189887e+02,-3.10436556e+00,3.74570060e-01,1.46398163e-01],
                        [-1.81869149e+01,-3.72054666e+01,-2.10918478e+02,3.13801503e+00,1.75261644e-02,-1.38106174e-01],
                        [-1.17375606e+02,-2.47069077e+01,-1.87936493e+02,-2.92199001e+00,1.55180620e-01,1.04162328e+00]])

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

objp = np.zeros((4 * 6, 3), np.float32)
objp[:, :2] = np.mgrid[0:6, 0:4].T.reshape(-1, 2)

objpoints = []  # 3d point in real world space
imgpoints = []  # 2d points in image plane

images = glob.glob('image/*.jpg')

# 命令发送
def servoWriteCmd(id, cmd, par1=None, par2=None):
    buf = bytearray(b'\x55\x55')
    try:
        len = 3  # 若命令是没有参数的话数据长度就是3
        buf1 = bytearray(b'')

        ## 对参数进行处理
        if par1 is not None:
            len += 2  # 数据长度加2
            par1 = 0xffff & par1
            buf1.extend([(0xff & par1), (0xff & (par1 >> 8))])  # 分低8位 高8位 放入缓存
        if par2 is not None:
            len += 2
            par2 = 0xffff & par2
            buf1.extend([(0xff & par2), (0xff & (par2 >> 8))])  # 分低8位 高8位 放入缓存

        buf.extend([(0xff & id), (0xff & len), (0xff & cmd)])  # 追加 id， 数据长度， 命令
        buf.extend(buf1)  # 追加参数

        ##计算校验和
        sum = 0x00
        for b in buf:  # 求和
            sum += b
        sum = sum - 0x55 - 0x55  # 去掉命令开头的两个 0x55
        sum = ~sum  # 取反
        buf.append(0xff & sum)  # 取低8位追加进缓存

        serialHandle.write(buf)  # 发送

    except Exception as e:
        print(e)


# 读取位置
def readPosition(id):
    serialHandle.flushInput()  # 清空接收缓存
    servoWriteCmd(id, command["POS_READ"])  # 发送读取位置命令
    time.sleep(0.00034)  # 小延时，等命令发送完毕。不知道是否能进行这么精确的延时的，但是修改这个值的确实会产生影响。
    # 实验测试调到这个值的时候效果最好
    time.sleep(0.005)  # 稍作延时，等待接收完毕
    count = serialHandle.inWaiting()  # 获取接收缓存中的字节数
    pos = None
    if count != 0:  # 如果接收到的数据不空
        recv_data = serialHandle.read(count)  # 读取接收到的数据
        if count == 8:  # 如果接收到的数据是8个字节（符合位置读取命令的返回数据的长度）
            if recv_data[0] == 0x55 and recv_data[1] == 0x55 and recv_data[4] == 0x1C:
                # 第一第二个字节等于0x55, 第5个字节是0x1C 就是 28 就是 位置读取命令的命令号
                pos = 0xffff & (recv_data[5] | (0xff00 & (recv_data[6] << 8)))  # 将接收到的字节数据拼接成完整的位置数据
                # 上面这小段代码我们简化了操作没有进行和校验，只要帧头和命令对了就认为数据无误

    return pos  # 返回读取到的位置

def init():
    servoWriteCmd(1, 1, 500, 1000)
    servoWriteCmd(2, 1, 750, 1000)
    servoWriteCmd(3, 1, 700, 1000)
    servoWriteCmd(4, 1, 700, 1000)
    servoWriteCmd(5, 1, 500, 1000)
    servoWriteCmd(6, 1, 500, 1000)

def getPosition(frame):
    img = cv2.resize(frame, (720, 480))
    copy = img.copy()

    lower = np.array([0, 96, 92])
    upper = np.array([24, 255, 255])
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower, upper)
    result = cv2.bitwise_and(img, img, mask=mask)
    canny = cv2.Canny(result, 80, 180)
    contours, hierarchy = cv2.findContours(canny, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    size = copy.shape

    maxPixels = 0
    maxContour = 0
    print(size)
    for i in range(len(contours)):
        single_rect = np.zeros((size[0], size[1]))
        fill_image = cv2.fillConvexPoly(single_rect, contours[i], 255)
        pixels = cv2.countNonZero(fill_image)
        peri = cv2.arcLength(contours[i], True)
        vertices = cv2.approxPolyDP(contours[i], peri*0.02, True)
        corners = len(vertices)
        if pixels > maxPixels and pixels > 500:
            maxPixels = pixels
            maxContour = i

    peri = cv2.arcLength(contours[maxContour], True)
    vertices = cv2.approxPolyDP(contours[maxContour], peri*0.02, True)
    corners = len(vertices)
    x, y, w, h = cv2.boundingRect(vertices)
    cv2.rectangle(copy, (x, y), (x + w, y + h), (0, 255, 0), 4)
    cv2.putText(copy, 'rectangle', (x, y-5), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 1)
    cv2.imshow('result', result)
    cv2.imshow('canny', canny)
    cv2.imshow('img', copy)
    # cv2.waitKey(0)
    position = [x+w/2, y+h/2]
    if maxPixels == 0:
        return None
    return position

def gripper2base(a1, a2, a3, a4):
    t1 = np.array([[math.cos(math.radians(a1 * 0.32 - 160)), -math.sin(math.radians(a1 * 0.32 - 160)), 0, 0],
                  [math.sin(math.radians(a1 * 0.32 - 160)), math.cos(math.radians(a1 * 0.32 - 160)), 0, 0],
                  [0, 0, 1, 0],
                  [0, 0, 0, 1]])

    t2 = np.array([[math.cos(math.radians(180 - a2 * 0.36)), -math.sin(math.radians(180 - a2 * 0.36)), 0, 0],
                  [0, 0, -1, -37],
                  [math.sin(math.radians(180 - a2 * 0.36)), math.cos(math.radians(180 - a2 * 0.36)), 0, 0],
                  [0, 0, 0, 1]])

    t3 = np.array([[math.cos(math.radians(a3 * 0.36 - 180)), -math.sin(math.radians(a3 * 0.36 - 180)), 0, 96],
                  [math.sin(math.radians(a3 * 0.36 - 180)), math.cos(math.radians(a3 * 0.36 - 180)), 0, 0],
                  [0, 0, 1, 0],
                  [0, 0, 0, 1]])
    t4 = np.array([[math.cos(math.radians(a4 * 0.36 - 180)), -math.sin(math.radians(a4 * 0.36 - 180)), 0, 96],
                  [math.sin(math.radians(a4 * 0.36 - 180)), math.cos(math.radians(a4 * 0.36 - 180)), 0, 0],
                  [0, 0, 1, 0],
                  [0, 0, 0, 1]])

    t5 = np.array([[1, 0, 0, 0],
                  [0, 0, -1, -55],
                  [0, 1, 0, 0],
                  [0, 0, 0, 1]])

    t = t1 @ t2 @ t3 @ t4 @ t5
    return t

def getBasePosition(position):
    pos3 = readPosition(3)  # 获取3号舵机的位置
    pos4 = readPosition(4)  # 获取4号舵机的位置
    pos5 = readPosition(5)  # 获取5号舵机的位置
    pos6 = readPosition(6)  # 获取6号舵机的位置
    H = np.zeros(shape=(4, 4), dtype=float)
    H = gripper2base(pos6, pos5, pos4, pos3)
    mtx, R = getEyeHand()
    mtx = mtx.T
    base = np.array([0, 0, 0])
    base = H * R * mtx * [position[0], position[1], 1]
    return base

def grasp(basePosition):
    pass
    # 进行运动学逆解，得到多个解，从多个解中找到最适合的解
    # 机械臂各个舵机旋转到指定角度
    # 机械臂末端到达指定位置，并略微调整位置
    # 夹取，抬升物体
    # 移动到稳定点

def place():
    pass
    # 第一步：云台转到朝向目标方向
    # 第二步：移到接近点
    # 第三步：移到目标点
    # 第四步：抬升
    # 第五步：放置
    # 第六步：移到稳定点

def getEyeHand():
    R_gripper2base = []
    t_gripper2base = []
    R_target2camera = []
    t_target2camera = []

    for frame in images:
        img = cv2.imread(frame)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        ret, corners = cv2.findChessboardCorners(gray, (6, 4), None)
        if ret == True:
            objpoints.append(objp)

            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)  # 在初步提取的角点信息上进一步提取亚像素信息，降低相机标定偏差
            imgpoints.append(corners)

            img = cv2.drawChessboardCorners(img, (6, 4), corners2, ret)

            cv2.imshow("img", img)
            cv2.waitKey(0)

    # mtx:相机内参矩阵
    # dist:畸变系数
    # rvecs:旋转向量
    # tvecs:平移向量
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
    # 求外参，对于每张图片外参矩阵都是不一样的，外参矩阵随着刚体位置的变换而变换

    # retval, rvec, tvec  = cv2.solvePnP(np.float32(objpoints), np.float32(imgpoints), mtx, dist)
    a = np.array([0, 0, 0, 1])
    for i in range(len(rvecs)):
        # 先利用solvePnP得到各个R_target2cam和T_target2cam
        # 利用gripper2base得到RT_gripper2base，然后将其分为R_gripper2base和T_gripper2base
        # 最后得到R,T = cv2.calibrateHandEye(R_gripper2base,T_gripper2base,R_target2cam,T_target2cam)
        # RT = np.column_stack((R,T))
        # RT = np.row_stack((RT, np.array([0, 0, 0, 1])))
        # 即可得到手眼标定最后的结果
        R_matrix, _ = cv2.Rodrigues(rvecs[i])
        R_target2camera.append(R_matrix)
        t_target2camera.append(tvecs[i])
        # Rt_matrix = np.concatenate((R_matrix, tvecs[i]), axis=1)
        # Rt_matrix = np.row_stack((Rt_matrix, a))
        # print(Rt_matrix)


    for i in range(len(gripperPose)):
        R_vector = gripperPose[i, 3:6]
        R_matrix, _ = cv2.Rodrigues(R_vector)
        R_gripper2base.append(R_matrix)
        t_gripper2base.append(gripperPose[i, 0:3].T)


    print("内参=", mtx)
    print("畸变系数=", dist)
    print("旋转向量=", rvecs)
    print("平移向量=", tvecs)

    newImage = cv2.imread('image/0.jpg')
    h, w = newImage.shape[:2]
    newCameraMtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
    dst = cv2.undistort(newImage, mtx, dist, None, newCameraMtx)
    x, y, w, h = roi
    dst = dst[y:y + h, x:x + w]

    # 求解重投影误差，越趋近于0越好
    mean_error = 0
    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
        error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
        mean_error += error
    print("total error: ", mean_error / len(objpoints))

    cv2.imshow('dst', dst)
    cv2.waitKey(0)

    # calibrateHandEye的结果是gripper2camera
    print("基坐标系的旋转矩阵与平移向量求解完毕————————————————")
    print("进入手眼标定函数————————————————————————————————")

    R, T = cv2.calibrateHandEye(R_gripper2base, t_gripper2base, R_target2camera, t_target2camera)#手眼标定

    print("手眼矩阵分解得到的旋转矩阵")
    print(R)
    print("\n")


    print("手眼矩阵分解得到的平移矩阵")
    print(T)

    RT=np.column_stack((R,T))
    RT = np.row_stack((RT, np.array([0, 0, 0, 1])))
    print("\n")
    print('相机相对于末端的变换矩阵为：')
    print(RT)
    return mtx, RT

if __name__=='__main__':
    init()
    cap = cv2.VideoCapture(0)
    while True:
        ret, frame = cap.read()
        if ret:
            position = getPosition(frame)
            if position is not None:
                basePosition = getBasePosition(position)
                print("basePosition", basePosition)
                grasp(basePosition)
                time.sleep(2)
                place()
                time.sleep(2)
            else:
                continue
