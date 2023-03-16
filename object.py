import cv2
import pyrealsense2 as rs
import numpy as np
intrinsics = [[909.52093506,   0,   630.67175293],
              [0, 907.04144287, 368.20245361],
              [0,           0,           1]]
mat = [[-9.96839521e-01,  7.88023563e-02, - 1.00576964e-02,  3.09193760e+01],
       [-7.89827349e-02, - 9.96694660e-01, 1.90127014e-02,  8.67676315e+01],
       [-8.52620663e-03, 1.97469965e-02,  9.99768653e-01, - 2.58392194e+02],
       [0.00000000e+00, 0.00000000e+00,  0.00000000e+00,  1.00000000e+00]]
xy = (0, 0)

mtx = [[-9.36577766e-01, -2.34645006e-01,  2.60314829e-01,  3.99353520e+01],
       [-3.45894427e-01, 7.38404973e-01, -5.78891303e-01, -4.60562420e+02],
       [-5.63838108e-02, -6.32218172e-01, -7.72736080e-01, 3.36979498e+02],
       [0, 0, 0, 1]]
new = [[-9.96962121e-01,  7.54158173e-02,  1.94674879e-02 ,-9.78117213e+01],
        [ 7.45148239e-02,  9.96272080e-01, -4.34681821e-02, -4.03826213e+02],
        [-2.26731031e-02, -4.18855146e-02, -9.98865123e-01 , 3.83424606e+02],
        [ 0.00000000e+00 , 0.00000000e+00,  0.00000000e+00,  1.00000000e+00]]
mtx = np.asarray(new)
pose = [[50.34186, -554.9487, 82.25433, 141.5474, -0.8874476, 24.1776]]


def take_picture():
    """ if not cam_realsense:
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
                #cv2.namedWindow("live", cv2.WINDOW_AUTOSIZE)  # 命名一個視窗，可不寫
                cv2.imshow("live", frame)

                cv2.waitKey(1000)
                return frame
        cv2.destroyAllWindows()
    else: """
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    cfg = pipeline.start(config)
    profile = cfg.get_stream(rs.stream.color)

    intr = profile.as_video_stream_profile().get_intrinsics()
    print(intr)
    intrinsics = np.asarray( [[intr.fx, 0, intr.ppx], [0,  intr.fy, intr.ppy], [0, 0, 1]])
    print(intrinsics)
    for i in range(0, 100):
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(
            depth_image, alpha=0.03), cv2.COLORMAP_JET)

    cv2.imshow("live", color_image)
    cv2.imwrite("test.jpg", color_image)
    cv2.imshow("depth", depth_colormap)
    cv2.imwrite("test_depth.jpg", depth_colormap)
    d = depth_frame.get_distance(355 ,147)
    print(d)
    cv2.waitKey(10)
    return color_image, depth_frame, intrinsics


def on_EVENT_LBUTTONDOWN(event, x, y, flags, param):  # 定義滑鼠事件
    if event == cv2.EVENT_LBUTTONDOWN:
        xy = (x, y)
        img2 = cv2.imread("./test.jpg")

        cv2.circle(img2, (x, y), 2, (0, 0, 255), thickness=-1)  # 將點標在圖上
        cv2.putText(img2, f"({x},{y})", (x, y), cv2.FONT_HERSHEY_PLAIN,
                    1, (0, 255, 0), thickness=1)
        cv2.imshow("Please click 3 times at photo(eyes and nose)", img2)
        print(x, y)


def click(color, depth):

    cv2.namedWindow("Please click 3 times at photo(eyes and nose)")
    cv2.setMouseCallback(
        "Please click 3 times at photo(eyes and nose)", on_EVENT_LBUTTONDOWN)
    cv2.imshow("Please click 3 times at photo(eyes and nose)", color)

    cv2.waitKey(0)
    cv2.destroyWindow('Please click 3 times at photo(eyes and nose)')


c, d, intrinsics = take_picture()
click(c, d)
new_x = np.multiply(355-intrinsics[0][2],0.341/intrinsics[0][0])
new_y = np.multiply(147-intrinsics[1][2],0.341/intrinsics[1][1])
print(new_x, new_y)

point = np.asarray([new_x, new_y, 341,1])

coord = mtx@point
print(coord)