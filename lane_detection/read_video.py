import cv2
import os
import time

def readVideo():
    cap = cv2.VideoCapture(os.getcwd() + "/lane_detection/origin_code(C version)/sample_data/track-s.mkv")
    fourcc = cv2.VideoWriter_fourcc(*'X264')
    out = cv2.VideoWriter('output.avi', fourcc, 20.0, (640,480))

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            print("Can't receive frame (stream end?). Exiting ... ")
            break

        out.write(frame)
        
        time.sleep(0.4)
        
        cv2.imshow('frame', frame)
        if cv2.waitKey(1) == ord('q'):
            break
        
        
    cap.release()
    out.release()
    cv2.destroyAllWindows()