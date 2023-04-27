#! /usr/bin/env python
# -- coding: utf-8 --
import rospy
from math import pi

import os, sys
import cv2
import cv2
from matplotlib import image
import concurrent.futures
import numpy as np
import time
import os

from std_msgs.msg import Float64, Int16

class PublishToState:
    def __init__(self):
        self.steer_pub = rospy.Publisher("lane_steer", Int16, queue_size=10)
        self.steer = Int16()

    def pub_erp(self, steer):
        self.steer = steer
        self.steer_pub.publish(self.steer)
        
class lane_detection:
    def __init__(self):
        self.prevCenter = []
        self.prevRawCenter = []
        self.prevDir = []
        self.frameLength = 0
        self.center = 0
        self.currentDirection = 0 # -1이면 왼쪽, 1이면 오른쪽
        self.continueToRevise = 0
        self.toReduce = 1
        self.sumError = 0
        
    def grey(self, image):
        return cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    
    def gauss(self, image):
        return cv2.GaussianBlur(image, (5, 5), 0)
    
    def canny(self, image):
        edges = cv2.Canny(image,50,150)
        return edges

    def region(self, image):
        height, width = image.shape
        
        # 제주도 가서 쓸 해상도로 최적화 해야함
        square = np.array([[(0, 340), (160, height // 2 + 30), (480, height // 2 + 30), (width, 340), (width, height), (0, height)]])

        mask = np.zeros_like(image)
        mask = cv2.fillPoly(mask, square, 255)
        mask = cv2.bitwise_and(image, mask)
        return mask

    def display_lines(self, image, lines):
        lines_image = np.zeros_like(image)
        #make sure array isn't empty
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line

                cv2.line(lines_image, (x1, y1), (x2, y2), (255, 0, 0), 10)
        return lines_image

    def average(self, image, lines):
        left = []
        right = []
        for line in lines:
            print(line)
            x1, y1, x2, y2 = line.reshape(4)
            #fit line to points, return slope and y-int
            parameters = np.polyfit((x1, x2), (y1, y2), 1)
            print(parameters)
            slope = parameters[0]
            y_int = parameters[1]
            #lines on the right have positive slope, and lines on the left have neg slope
            if slope < 0:
                left.append((slope, y_int))
            else:
                right.append((slope, y_int))
        #takes average among all the columns (column0: slope, column1: y_int)
        right_avg = np.average(right, axis=0)
        left_avg = np.average(left, axis=0)
        #create lines based on averages calculates
        left_line = self.make_points(image, left_avg)
        right_line = self.make_points(image, right_avg)
        return np.array([left_line, right_line])

    def make_points(self, image, average):
        print(average)
        slope, y_int = average
        y1 = image.shape[0]
        #how long we want our lines to be --> 3/5 the size of the image
        y2 = int(y1 * (3/5))
        #determine algebraically
        x1 = int((y1 - y_int) // slope)
        x2 = int((y2 - y_int) // slope)
        
        x1 = min(x1, 100000)
        y1 = min(y1, 100000)
        x2 = min(x2, 100000)
        y2 = min(y2, 100000)
        x1 = max(x1, -100000)
        y1 = max(y1, -100000)
        x2 = max(x2, -100000)
        y2 = max(y2, -100000)
        return np.array([x1, y1, x2, y2])

    def high_contrast(self, img):
        lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8)) 
        cl = clahe.apply(l)
        limg = cv2.merge((cl, a, b))
        final = cv2.cvtColor(limg, cv2.COLOR_LAB2BGR)
        
        return final


    def linearFunction(self, k, x, x1, y1):
        print("k ::::::::::::::::::", k)
        return k *(x - x1) + y1

    def extendLine(self, pt1, pt2):
        if pt1[0] - pt2[0] != 0:
            dx = pt1[0] - pt2[0]    
            dy = pt1[1] - pt2[1]
            
            k = dy / dx
            
            x = 320
            y = int(self.linearFunction(k, x, pt1[0], pt1[1])) 
        else:
            k = 0
            return pt1[0], pt1[1], k
        
        return x, y, k

    def outlinersIQR(self, data):
        X = []
        Y = []
        
        for i in range(len(data)):
            X.append(data[i][0])
            Y.append(data[i][1])
        
        q25, q75 = np.quantile(Y, 0.25), np.quantile(Y, 0.75)          
        iqr = q75 - q25   
        cut_off = iqr # * 1.5          
        lower, upper = q25 - cut_off, q75 + cut_off     
    
        newX = []    
        newY = []
        
        for i in range(len(Y)):
            if lower < Y[i] < upper:
                newY.append(Y[i])
        
        # print("newY ::::::::::::::", newY)
        
        count = 0 
        
        for i in range(len(Y)):
            if Y[i] not in newY:
                count += 1
                # print(Y[i], " not in list")
            else:
                newX.append(X[i])

        if count == 0: 
            pass
            # print("############################################")
        else:
            print("Remove Noises")
            
        return newX, newY

    def divideLine(self, img, copy):
        
        left = []
        right = []
        h, w = img.shape
        
        for y in range(h // 3 * 2, h):
            for x in range(w // 6):
                if img[y][x] == 255:
                    left.append((x, y))
                    
            for x in range(w // 6 * 5, w):
                if img[y][x] == 255:
                        right.append((x, y))
        
        if len(right) != 0 or len(left) != 0:
    

            # print("len(left)", len(left))
            # print("len(right)", len(right))
            
            IGNORED = 100
            
            if len(left) > IGNORED:
                # print("left[:][0]", left[:])
                LX, LY = self.outlinersIQR(left)

                # LX = left[:][0]
                # LY = left[:][1]
            else:
                LX = [0]
                LY = [300]
                
            if len(right) > IGNORED:
                RX, RY = self.outlinersIQR(right)
            else:
                RX = [640]
                RY = [300]
            
            avgLX = int(np.mean(LX, axis=0))
            avgLY = int(np.mean(LY, axis=0))
            avgRX = int(np.mean(RX, axis=0))
            avgRY = int(np.mean(RY, axis=0))
            
            # if 450 < LY[0] < 640:
            for x, y in zip(LX, LY):
                cv2.circle(copy, (x, y), 2, (0, 0, 255), -1)
            
            # if 450 < RY[-1] < 640:
            for x, y in zip(RX, RY):
                cv2.circle(copy, (x, y), 2, (0, 0, 255), -1)
            
            # 변수 미리 초기화
            lpt1, lpt2, rpt1, rpt2 = [0, 0, 0, 0]
            lk, rk = [0, 0]
            
            if len(LX) < IGNORED and len(RX) < IGNORED: 
                self.currentDirection = 0 
                #########################
                print("Detected Nothing") 
                #########################
            elif len(LX) < IGNORED : # or RY[-1] < 450
                rpt1 = (min(RX), min(RY))
                rpt2 = (max(RX), max(RY))
                rx, ry, rk = self.extendLine(rpt1, rpt2)
                lx, ly, lk = [RX[0], RY[0], 0]
                
                cv2.line(copy, rpt1, (rx, ry), (0, 0, 0), 4)
            elif len(RX) < IGNORED : # or LY[0] < 450
                lpt1 = (min(LX), max(LY))
                lpt2 = (max(LX), min(LY))
                lx, ly, lk = self.extendLine(lpt1, lpt2)
                rx, ry, rk = [LX[0], LY[0], 0]
                
                cv2.line(copy, lpt1, (lx, ly), (0, 0, 0), 4)
            else:
                lpt1 = (min(LX), max(LY))
                lpt2 = (max(LX), min(LY))
                lx, ly, lk = self.extendLine(lpt1, lpt2)
                
                rpt1 = (min(RX), min(RY))
                rpt2 = (max(RX), max(RY))
                rx, ry, rk = self.extendLine(rpt1, rpt2)
                
                cv2.line(copy, lpt1, (lx, ly), (0, 0, 0), 4)
                cv2.line(copy, rpt1, (rx, ry), (0, 0, 0), 4)
            
            
            # center = [(avgLX + avgRX) // 2, (avgLY + avgRY) // 2]
            center = [w // 2, h // 2]
            
            # K = [lk, rk]
            # 보정 계수 조정
            a = 20
            b = 7 
            
            if lk == 0 and rk == 0:
                #########################
                print("Detected Nothing") 
                #########################
                self.currentDirection = 0
                center[0] += 0
            elif lk == 0:
                # print("Turn Left : ", a * (abs(rk) - abs(lk)))
                self.currentDirection = -1
                center[0] -= int(a * (abs(rk) - abs(lk)))
            elif rk == 0:
                # print("Turn Right : ", a * (abs(lk) - abs(rk)))
                self.currentDirection = 1
                center[0] += int(a * (abs(lk) - abs(rk)))
            else:
                print("abs(lk) :", abs(lk))
                print("abs(rk) :", abs(rk))
                if abs(lk) < abs(rk):
                    # print("Turn Right : ", b * (abs(lk) - abs(rk)))
                    self.currentDirection = 1
                    center[0] += int(b * abs(abs(lk) - abs(rk)))
                else:
                    # print("Turn Left : ", b * (abs(rk) - abs(lk)))
                    self.currentDirection = -1
                    center[0] -= int(b * abs(abs(rk) - abs(lk)))
                    
            print("center :", center)
                
                
            if center[0] > w // 2:
                print("Turn Right : ", abs(center[0] - w // 2))
            else:
                print("Turn Left : ", abs(center[0] - w // 2))
                
            cv2.line(copy, (w // 2, h), (w // 2, 0), (255, 255, 255), 2)
            cv2.circle(copy, tuple(center), 10, (0, 255, 0), -1)
            
            return center[0] - w // 2 # 최종 steer값 -면 왼쪽, +면 오른쪽
        
        return 0.0
    
    def run(self):    
        cap = cv2.VideoCapture(2) #웹캠으로 받아오기, 2번 사용하면 됨
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,640)  #해상도 조절해주기,웹캠사용시 필요
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT,480)
        # cap = os.getcwd() #현재 경로 헷갈릴때 확인하기(비디오 넣어서 확인할때만!)
        # print(cap)
        # cap = cv2.VideoCapture(os.getcwd() + "/catkin_ws/src/2023-JEJU-AA1-5-main/src/missions/track-s.mkv")
        # fourcc = cv2.VideoWriter_fourcc(*'X264')
        # out = cv2.VideoWriter('output.avi', fourcc, 20.0, (640,480))

        

        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                print("Can't receive frame (stream end?). Exiting ... ")
                break
            
            current = time.time()
            h, w, _= frame.shape
            
            # print("shape :", frame.shape)
            # frame = high_contrast(frame)
            copy = frame
            gaus = self.gauss(frame)
            edges = cv2.Canny(gaus,50,150)
            # edges = region(edges)
            edges = self.region(edges)
            
            steer = self.divideLine(edges, copy)
            print("steer :", steer)
            cv2.imshow('edges', edges)
            cv2.imshow('copy', copy)

            print("time :", time.time() - current, "s")
            # cv2.imshow('frame', lanes)
            # time.sleep(1)
            # time.sleep(0.00000005)
            if cv2.waitKey(1) == ord('q'):
                break
                

        cap.release()
        # out.release()
        cv2.destroyAllWindows()
        
def main():
    rospy.init_node('lane_node', anonymous=True)
    Lane = lane_detection()
    pub = PublishToState()
    
    cap = cv2.VideoCapture(2) #웹캠으로 받아오기, 2번 사용하면 됨
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,640)  #해상도 조절해주기,웹캠사용시 필요
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT,480)
    
    speed = 50

    while not rospy.is_shutdown():
        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                print("Can't receive frame (stream end?). Exiting ... ")
                break
            
            current = time.time()
            h, w, _= frame.shape
            
            copy = frame
            gaus = Lane.gauss(frame)
            edges = cv2.Canny(gaus,50,150)
            edges = Lane.region(edges)
            
            steer = Lane.divideLine(edges, copy)
            print("steer :", steer)
            cv2.imshow('edges', edges)
            cv2.imshow('copy', copy)

            print("time :", time.time() - current, "s")
            
            if cv2.waitKey(1) == ord('q'):
                break
            
            steer = np.clip(steer, -22, 22)
            pub.pub_erp(steer)
            
        cap.release()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()

