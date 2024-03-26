import cv2
import numpy as np
import pyrealsense2.pyrealsense2 as rs
import time

def start_camera():
    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start streaming
    pipeline.start(config)

    return pipeline, config, 640  # width of the image frame

def detect_green_blob(pipeline, config, width):
    # Wait for a coherent pair of frames: depth and color
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    if not color_frame:
        return 3  # No blob detected

    # Convert images to numpy arrays
    color_image = np.asanyarray(color_frame.get_data())

    # Convert BGR image to HSV
    hsv_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)

    # Define color range for green color in HSV
    lower_green = np.array([35, 100, 100])
    upper_green = np.array([85, 255, 255])

    # Threshold the HSV image to get only green colors
    mask = cv2.inRange(hsv_image, lower_green, upper_green)

    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Draw contours and calculate center coordinates of the blobs
    for contour in contours:
        area = cv2.contourArea(contour)
        # Filter out small noise by comparing the area to a threshold
        if area > 50:  # Lower the threshold to detect smaller blobs
            # Calculate center coordinates of the contour
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                center_coordinates = (cx, cy)

                # Draw a circle at the center of the blob
                cv2.circle(color_image, center_coordinates, 5, (0, 255, 0), -1)

                # Display the coordinates as text values
                cv2.putText(color_image, f"({cx}, {cy})", (cx + 10, cy - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                # Compare the center of the blob to the center of the image
                image_center = width // 2
                if abs(image_center - cx) <= 5:
                    return 0  # Blob is in the center
                elif cx < image_center:
                    return 1  # Blob is to the left
                else:
                    return 2  # Blob is to the right

    # Show images
    cv2.imshow('Detected Green Blob', color_image)
    
    return 3  # No blob detected

def stop_camera(pipeline):
    # Stop streaming
    pipeline.stop()
    cv2.destroyAllWindows()

def main():
    try:
        pipeline, config, width = start_camera()
        while True:
            result = detect_green_blob(pipeline, config, width)
            print(result)
    except KeyboardInterrupt:
        stop_camera(pipeline)

if __name__ == "__main__":
    main()