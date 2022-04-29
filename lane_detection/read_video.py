#!/usr/local/bin/python3
#-*- coding: utf-8 -*-
import cv2
import os
import time

def readVideo():
    
    cap = cv2.VideoCapture(2)
    # cap = cv2.VideoCapture(os.getcwd() + "/lane_detection/origin_code(C version)/sample_data/track-s.mkv")
    # fourcc = cv2.VideoWriter_fourcc(*'X264')
    # out = cv2.VideoWriter('output.avi', fourcc, 20.0, (640,480))

    while cap.isOpened():
        current = time.time()
        ret, frame = cap.read()
        if not ret:
            print("Can't receive frame (stream end?). Exiting ... ")
            break

        # out.write(frame)
        
        cv2.imshow('frame', frame)
        print(time.time() - current, "s")
        
        
        if cv2.waitKey(1) == ord('q'):
            break
        
        
    cap.release()
    # out.release()
    cv2.destroyAllWindows()
    
readVideo()