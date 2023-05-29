import cv2
import numpy as np

cap = cv2.VideoCapture(0)

template_image = cv2.imread('template.png')
template_gray = cv2.cvtColor(template_image, cv2.COLOR_BGR2GRAY)

scale_factors = [.15, .10, .05] 

threshold = 0.70

while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    boxes = []
    for scale in scale_factors:
        resized_template = cv2.resize(template_image, None, fx=scale, fy=scale)
        result = cv2.matchTemplate(frame, resized_template, cv2.TM_CCOEFF_NORMED)
        locations = np.where(result >= threshold)

        for loc in zip(*locations[::-1]):
            top_left = loc
            bottom_right = (top_left[0] + resized_template.shape[1], top_left[1] + resized_template.shape[0])
            boxes.append([top_left[0], top_left[1], bottom_right[0], bottom_right[1]])

    boxes = np.array(boxes)
    indices = cv2.dnn.NMSBoxes(boxes.tolist(), [1]*len(boxes), threshold, 0.9)
    detected_templates = len(indices)  # Count the number of non-overlapping detections

    for i in indices:
        i = i.item(0)
        box = boxes[i]
        top_left = (box[0], box[1])
        bottom_right = (box[2], box[3])
        cv2.rectangle(frame, top_left, bottom_right, (0, 255, 0), 2)

        # Compute the center coordinates of the detected template
        center_x = int((top_left[0] + bottom_right[0]) / 2)
        center_y = int((top_left[1] + bottom_right[1]) / 2)

        # Display the coordinates of the center of the detected template on the frame
        cv2.putText(frame, f"Center: ({center_x}, {center_y})", (center_x, center_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

    # Display the number of detected templates on the frame
    cv2.putText(frame, f"Detected templates: {detected_templates}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    cv2.imshow('Result', frame)

    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()