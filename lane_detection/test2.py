import cv2
import cv2
from matplotlib import image
from matplotlib import pyplot as plt
import numpy as np
import time
import os

def region(image):
    height, width, _ = image.shape
    # triangle = np.array([
    #                    [(100, height), (475, 325), (width, height)]
    #                    ])
    square = np.array([[(0, 300), (160, height // 2), (480, height // 2), (width, 300), (width, height), (0, height)]])
    # trianle = np.array([[(width // 2, 200), ]])
    mask = np.zeros_like(image)
    mask = cv2.fillPoly(mask, square, 255)
    
    mask = cv2.bitwise_and(image, mask)
    return mask

cap = cv2.VideoCapture(os.getcwd() + "/lane_detection/origin_code(C version)/sample_data/track-s.mkv")
fourcc = cv2.VideoWriter_fourcc(*'X264')
out = cv2.VideoWriter('output.avi', fourcc, 20.0, (640,480))



while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        print("Can't receive frame (stream end?). Exiting ... ")
        break
    
    h, w, _= frame.shape
    
    print("shape :", frame.shape)

    copy = frame

    
    isolated = region(frame)
    # isolated = cv2.cvtColor(isolated, cv2.COLOR_BGR2HSV)
    # lines = cv2.HoughLinesP(isolated, 2, np.pi/180, 100, np.array([]), minLineLength=40, maxLineGap=5)

    # frame = canny(frame)
    cv2.imshow('edges', isolated)
    # cv2.line(frame, (100, 500), (400, 100), (0, 0, 255), 5)
    cv2.imshow('frame', frame)
    # time.sleep(1)
    time.sleep(0.5)
    if cv2.waitKey(1) == ord('q'):
        break
        

cap.release()
out.release()
cv2.destroyAllWindows()