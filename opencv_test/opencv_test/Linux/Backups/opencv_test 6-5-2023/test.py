import cv2
import numpy as np
import pyrealsense2.pyrealsense2 as rs

# Step 1: Setup for RealSense
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

# Step 2: Load the template image
template_image = cv2.imread('template.png')
template_gray = cv2.cvtColor(template_image, cv2.COLOR_BGR2GRAY)

# Step 3: Define scale factors
scale_factors = [.15, .25, .5, .75, 1, 1.25, 1.5]  # Adjust the scale factors as needed

# Step 4: Define the threshold value
threshold = 0.65  # Adjust the threshold value (between 0 and 1) as needed

while True:
    # Step 5: Read the next frame from the RealSense camera
    frames = pipeline.wait_for_frames()


    # Step 6: Convert the frame to grayscale
    frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Step 7-11: Perform template matching for each scale factor
    for scale in scale_factors:
        # Resize template
        resized_template = cv2.resize(template_image, None, fx=scale, fy=scale)

        # Perform template matching
        result = cv2.matchTemplate(frame, resized_template, cv2.TM_CCOEFF_NORMED)

        # Find match locations where the result is above the threshold
        locations = np.where(result >= threshold)

        # Iterate over the valid match locations
        for loc in zip(*locations[::-1]):
            top_left = loc
            bottom_right = (top_left[0] + resized_template.shape[1], top_left[1] + resized_template.shape[0])
            cv2.rectangle(frame, top_left, bottom_right, (0, 255, 0), 2)

    # Step 12: Display the result
    cv2.imshow('Result', frame)

    # Exit loop if 'q' is pressed
    if cv2.waitKey(1) == ord('q'):
        break

# Step 13: Stop the pipeline and close windows
pipeline.stop()
cv2.destroyAllWindows()