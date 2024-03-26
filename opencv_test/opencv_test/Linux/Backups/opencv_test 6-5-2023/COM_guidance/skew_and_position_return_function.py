import tkinter as tk
import serial
import time
import cv2
import numpy as np
import pyrealsense2.pyrealsense2 as rs
import time

def initialize_camera():
    """
    Initialize the camera and return pipeline, align object, and template image.
    """
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

    return pipeline, align, template_image


def detect_rotated_template(pipeline, align, template_image, threshold=0.70, scale = .10):
    """
    Detect a template rotated 90 degrees and return:
    - 0 if the template is within 5 pixels of the center of the camera
    - 1 if the template is to the left of the center
    - 2 if the template is to the right of the center
    """
    # Rotate the template image 90 degrees
    rotated_template = cv2.rotate(cv2.resize(template_image, None, fx=scale, fy=scale), cv2.ROTATE_90_CLOCKWISE)
    
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)
    color_frame = aligned_frames.get_color_frame()
    color_image = np.asanyarray(color_frame.get_data())
    
    # Find the best match
    result = cv2.matchTemplate(color_image, rotated_template, cv2.TM_CCOEFF_NORMED)
    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)
    
    # If the match is strong enough
    if max_val >= threshold:
        # Determine the horizontal position of the match
        match_center = max_loc[0] + rotated_template.shape[1] / 2
        
        camera_center = color_image.shape[1] / 2
        if abs(match_center - camera_center) <= 5:
            return 0
        elif match_center < camera_center:
            return 1
        else:
            return 2
    else:
        return None

def calculate_skew(pipeline, align, template_image, scale_factors = [.15, .10], threshold = 0.70):
    """
    Calculate the skew of the camera and return it.
    """
    # Function for calculating skew angle
    def get_camera_skew(centers_and_depths):
        dx = 0.36195
        if (centers_and_depths[0][0] < centers_and_depths[1][0]):
            dz = centers_and_depths[1][1] - centers_and_depths[0][1]
        elif(centers_and_depths[0][0] > centers_and_depths[1][0]):
            dz = centers_and_depths[0][1] - centers_and_depths[1][1]
        else:
            dz = 0    
        skew_angle = np.arctan2(dx, dz) 
        return np.degrees(skew_angle) - 90

    # Capture frames and calculate skew
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)
    depth_frame = aligned_frames.get_depth_frame()
    color_frame = aligned_frames.get_color_frame()
    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())

    centers_and_depths = []
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

    for i in indices:
        i = i.item(0)
        box = boxes[i]
        top_left = (box[0], box[1])
        bottom_right = (box[2], box[3])
        center_x = int((top_left[0] + bottom_right[0]) / 2)
        depth = depth_frame.get_distance(center_x, int((top_left[1] + bottom_right[1]) / 2))
        centers_and_depths.append((center_x, depth))

    if len(centers_and_depths) >= 2:
        skew_angle = get_camera_skew(centers_and_depths)
        return skew_angle
    else:
        return None

def close_camera(pipeline):
    """
    Stop the camera and close any open windows.
    """
    pipeline.stop()
    cv2.destroyAllWindows()

def main():
    # Initialize the camera
    pipeline, align, template_image = initialize_camera()

    try:
        while True:
            # Calculate the skew
            skew = calculate_skew(pipeline, align, template_image)
            if skew is not None:
                print(f"Camera skew: {skew} degrees")
            else:
                print("Insufficient templates detected to calculate skew.")
            
            # Detect the rotated template

            rotated_template_position = detect_rotated_template(pipeline, align, template_image)            
            if rotated_template_position is not None:
                print(f"Rotated template position: {rotated_template_position}")
            else:
                print("Rotated template not detected.")

            # Add delay or quit loop logic here

    except KeyboardInterrupt: # Or any other exception that you want to handle
        print("Interrupted...")

    finally:
        # Close the camera
        close_camera(pipeline)



if __name__ == "__main__":
    main()

