import tkinter as tk
import serial
import time
from lidar import get_distance
import cv2
import numpy as np
import pyrealsense2.pyrealsense2 as rs
import time

# Initialize Intel RealSense
pipeline = rs.pipeline()
config = rs.config()

# Enable color and depth streams
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

# Create align object to align depth frames to color frame
align_to = rs.stream.color
align = rs.align(align_to)

# Start streaming
profile = pipeline.start(config)

template_image = cv2.imread('template.png')
scale_factors = [.15, .10] 
threshold = 0.70

try:
    while True:
        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        
        # Align the depth frame to color frame
        aligned_frames = align.process(frames)

        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        boxes = []
        for scale in scale_factors:
            resized_template = cv2.resize(template_image, None, fx=scale, fy=scale)
            result = cv2.matchTemplate(color_image, resized_template, cv2.TM_CCOEFF_NORMED)
            locations = np.where(result >= threshold)

            for loc in zip(*locations[::-1]):
                top_left = loc
                bottom_right = (top_left[0] + resized_template.shape[1], top_left[1] + resized_template.shape[0])
                boxes.append([top_left[0], top_left[1], bottom_right[0], bottom_right[1]])

        boxes = np.array(boxes)
        indices = cv2.dnn.NMSBoxes(boxes.tolist(), [1]*len(boxes), threshold, 0.9)
        detected_templates = len(indices)

        for i in indices:
            i = i.item(0)
            box = boxes[i]
            top_left = (box[0], box[1])
            bottom_right = (box[2], box[3])
            cv2.rectangle(color_image, top_left, bottom_right, (0, 255, 0), 2)

            # Compute the center coordinates of the detected template
            center_x = int((top_left[0] + bottom_right[0]) / 2)
            center_y = int((top_left[1] + bottom_right[1]) / 2)

            # Get the depth at the center of the detected template
            depth = depth_frame.get_distance(center_x, center_y)
            print(f"Depth at center: {depth} meters, x-coordinate: {center_x}")

            # Display the coordinates of the center of the detected template on the frame
            cv2.putText(color_image, f"Center: ({center_x}, {center_y})", (center_x, center_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            
            # Display the depth of the center of the detected template on the frame
            cv2.putText(color_image, f"Depth: {depth}", (center_x, center_y - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            
        # Display the number of detected templates on the frame
        cv2.putText(color_image, f"Detected templates: {detected_templates}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        cv2.imshow('RealSense', color_image)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # Stop streaming
    pipeline.stop()
    cv2.destroyAllWindows()
