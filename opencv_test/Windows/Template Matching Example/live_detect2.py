import cv2
import numpy as np

# Step 1: Initialize video capture from the laptop camera
cap = cv2.VideoCapture(0)

# Step 2: Load the template image
template_image = cv2.imread('template.png')
template_gray = cv2.cvtColor(template_image, cv2.COLOR_BGR2GRAY)

# Step 3: Initialize SIFT detector
sift = cv2.SIFT_create()

# Step 4: Compute keypoints and descriptors for the template
template_keypoints, template_descriptors = sift.detectAndCompute(template_gray, None)

# Step 5: Initialize FLANN matcher
FLANN_INDEX_KDTREE = 1
index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
search_params = dict(checks = 50)
flann = cv2.FlannBasedMatcher(index_params, search_params)

while True:
    # Step 6: Read the next frame from the camera
    ret, frame = cap.read()
    if not ret:
        break

    # Step 7: Convert the frame to grayscale
    frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Step 8: Compute keypoints and descriptors for the frame
    frame_keypoints, frame_descriptors = sift.detectAndCompute(frame_gray, None)

    # Step 9: Match descriptors
    matches = flann.knnMatch(template_descriptors, frame_descriptors, k=2)

    # Step 10: Filter matches using the Lowe's ratio test
    good_matches = [m for m,n in matches if m.distance < 0.7*n.distance]

    # Step 11: Draw only good matches
    result = cv2.drawMatches(template_image, template_keypoints, frame, frame_keypoints, good_matches, None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)

    # Step 12: Display the result
    cv2.imshow('Result', result)

    # Exit loop if 'q' is pressed
    if cv2.waitKey(1) == ord('q'):
        break

# Step 13: Release the video capture and close windows
cap.release()
cv2.destroyAllWindows()