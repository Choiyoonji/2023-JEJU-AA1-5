from multiprocessing.connection import wait
import cv2
import numpy as np

x = np.zeros((480, 640, 3), np.uint8)

        

# cv2.circle(x, (0, 0), 50, (0, 255, 255))
cv2.line(x, (-710, 770), (1022, -229), (0, 255, 255), 10)
# cv2.circle(x, (50, 50), 50, (0, 255, 255))
# cv2.circle(x, (50, -50), 50, (0, 255, 255))

cv2.imshow("SS", x)
cv2.waitKey(0)