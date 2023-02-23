import pyrealsense2 as rs

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth,640,480,rs.format.z16,30)
config.enable_stream(rs.stream.color, 640,480,rs.format.bgr8,30)
profile = pipeline.start(config)
frames = pipeline.wait_for_frames()
depth = frames.get_depth_frame()
color = frames.get_color_frame()

depth_profile = depth.get_profile()
print(depth_profile)
color_profile = color.get_profile()

cvsprofile= rs.video_stream_profile(color_profile)
dvsprofile = rs.video_stream_profile(depth_profile)

color_intrin = cvsprofile.get_intrinsics()

print(color_intrin)

extrin = depth_profile.get_extrinsics_to(color_profile)
print(extrin)