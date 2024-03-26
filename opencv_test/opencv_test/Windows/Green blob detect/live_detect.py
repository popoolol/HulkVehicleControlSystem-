import cv2
import numpy as np

def detect_green_blob():
    # Access the webcam feed
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Could not open webcam")
        exit(0)

    while True:
        # Read frame from the webcam
        ret, frame = cap.read()

        if not ret:
            break

        # Convert BGR image to HSV
        hsv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

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
                    cv2.circle(frame, center_coordinates, 5, (0, 255, 0), -1)

                    # Display the coordinates as text values
                    cv2.putText(frame, f"({cx}, {cy})", (cx + 10, cy - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Display the resulting frame
        cv2.imshow('Detected Green Blob', frame)

        # Break the loop on 'q' key press
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the webcam and close windows
    cap.release()
    cv2.destroyAllWindows()

detect_green_blob()
