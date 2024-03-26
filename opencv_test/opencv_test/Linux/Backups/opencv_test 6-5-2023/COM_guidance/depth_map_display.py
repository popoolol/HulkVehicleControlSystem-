import tkinter as tk
import serial
import time
from lidar import get_distance
import cv2
import numpy as np
import pyrealsense2 as rs
import time

# Initialize Intel RealSense
pipeline = rs.pipeline()
config = rs.config()

# Enable color and depth streams
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

# Start streaming
profile = pipeline.start(config)

# Getting the depth sensor's depth_scale (using the first depth sensor)
depth_sensor = profile.get_device().first_depth_sensor()

# Enable High Accuracy preset
depth_sensor.set_option(rs.option.visual_preset, 3)  # rs.option.visual_preset equals to 3 for 'High Accuracy'

# Create colorizer object
colorizer = rs.colorizer()

try:
    while True:
        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        
        # Colorize the depth frame
        colorized_depth = np.asanyarray(colorizer.colorize(depth_frame).get_data())
        
        # Display the colorized depth map
        cv2.imshow('RealSense Depth Map', colorized_depth)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # Stop streaming
    pipeline.stop()
    cv2.destroyAllWindows()
