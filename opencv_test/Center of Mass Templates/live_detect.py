import cv2
import numpy as np



image = cv2.imread('test.png')


gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

template = cv2.imread('symbol_to_detect1.png', 0)

result = cv2.matchTemplate(gray, template, cv2.TM_CCOEFF_NORMED)

threshold = .5  # Adjust this threshold value as needed
locations = np.where(result >= threshold)

symbol_width, symbol_height = template.shape[::-1]  # Get the width and height of the template
for pt in zip(*locations[::-1]):
    cv2.rectangle(image, pt, (pt[0] + symbol_width, pt[1] + symbol_height), (0, 255, 0), 2)

cv2.imshow('Symbol Detection Result', image)
cv2.waitKey(0)
cv2.destroyAllWindows()