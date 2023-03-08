import pyrealsense2 as rs
import time
import matplotlib.pyplot as plt
import numpy as np 
import cv2
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 1280, 720, rs.format.rgb8, 30) 

cfg = pipeline.start(config)
profile = cfg.get_stream(rs.stream.color)

intr = profile.as_video_stream_profile().get_intrinsics()
print(intr)
print(intr.coeffs)

time.sleep(1)
frames = pipeline.wait_for_frames()
time.sleep(3)
depth_frame = frames.get_depth_frame()
color_frame = frames.get_color_frame()
 
    # Convert images to numpy arrays
depth_image = np.asanyarray(depth_frame.get_data())
color_image = np.asanyarray(color_frame.get_data())


# # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
color_image = cv2.cvtColor(color_image,cv2.COLOR_BGR2RGB)
# # Stack both images horizontally
images = np.vstack((color_image, depth_colormap))

# # Show images
cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
cv2.imshow('RealSense', images)
cv2.waitKey(0)
