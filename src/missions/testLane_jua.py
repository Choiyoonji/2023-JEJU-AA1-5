#! /usr/bin/env python
# -- coding: utf-8 --
import rospy
import cv2
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import image
import os
import time
from std_msgs.msg import Int16

path = '/home/choiyoonji/catkin_ws/src/2023-JEJU-AA1-5/src/missions/video1502937659 (1).mov'

class PublishToState:
    def __init__(self):
        self.steer_pub = rospy.Publisher("lane_steer", Int16, queue_size=1)
        self.steer = Int16()

    def pub_erp(self, steer):
        self.steer = steer
        self.steer_pub.publish(self.steer)
        
class Lane_detection:
    def __init__(self):
        self.image = None
    
    def region_of_interest(self, image, vertices, color3=(255,255,255), color1=255): # ROI 셋팅

        mask = np.zeros_like(image) # mask = img와 같은 크기의 빈 이미지
        
        if len(image.shape) > 2: # Color 이미지(3채널)라면 :
            color = color3
        else: # 흑백 이미지(1채널)라면 :
            color = color1
            
        # vertices에 정한 점들로 이뤄진 다각형부분(ROI 설정부분)을 color로 채움 
        cv2.fillPoly(mask, vertices, color)
        
        # 이미지와 color로 채워진 ROI를 합침
        ROI_image = cv2.bitwise_and(image, mask)
        return ROI_image

    def mark_img(self, img, mark, image, blue_threshold=200, green_threshold=200, red_threshold=200): # 흰색 차선 찾기
        # print('image : ', image)
        #  BGR 제한 값
        bgr_threshold = [blue_threshold, green_threshold, red_threshold]

        # BGR 제한 값보다 작으면 검은색으로
        thresholds = (image[:,:,0] < bgr_threshold[0]) \
                    | (image[:,:,1] < bgr_threshold[1]) \
                    | (image[:,:,2] < bgr_threshold[2])
        # print('thresholds : ', thresholds)
        mark[thresholds] = [0,0,0]
        return mark

    def get_lane_center(self, image):
        # 흰색 픽셀의 좌표 구하기
        white_pixels = np.argwhere(image[:,:,2] > 230)

        # 흰색 픽셀이 없으면 None 반환
        if len(white_pixels) == 0:
            return None

        # 흰색 픽셀의 x, y 좌표 평균값 구하기
        x_mean = np.mean(white_pixels[:,1])
        y_mean = np.mean(white_pixels[:,0])
        
        # if(250<x_mean<450 and 90 <y_mean <170) :
        #     x_mean = 300
        #     y_mean = 125
        #     return(int(x_mean) , int(y_mean))
        

        return (int(x_mean), int(y_mean)), white_pixels

    def sliding_window(self, img, nwindows=15, margin=90, minpix=5):
        histogram = np.sum(img[img.shape[0] // 2:, :], axis=0)

        out_img = np.dstack((img, img, img))

        midpoint = np.int64(320)  # (path) -> 350, (2) -> 500
        leftx_base = np.argmax(histogram[:midpoint])
        
        rightx_base = np.argmax(histogram[midpoint:]) + midpoint

        window_height = np.int64(img.shape[0] // nwindows)

        nonzero = img.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])

        leftx_current = leftx_base 
        rightx_current = rightx_base + 50 #원래 mid 400에 140, mid 320 에 100,이 윈도우 검출 더 잘됨

        left_lane_inds = []
        right_lane_inds = []
        
        last_window_left = 0
        last_window_right = 0

        for window in range(nwindows):
            win_y_low = img.shape[0] - (window + 1) * window_height
            win_y_high = img.shape[0] - window * window_height
            
            # if leftx_current 
            
            win_xleft_low = leftx_current - margin
            win_xleft_high = leftx_current + margin
            win_xright_low = rightx_current - margin
            win_xright_high = rightx_current + margin
            
            last_window_left += leftx_current
            last_window_right += rightx_current

            cv2.rectangle(out_img, (win_xleft_low, win_y_low), (win_xleft_high, win_y_high), (0, 255, 0), 2)
            cv2.rectangle(out_img, (win_xright_low, win_y_low), (win_xright_high, win_y_high), (0, 255, 0), 2)

            good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                            (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
            good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                            (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]

            left_lane_inds.append(good_left_inds)
            right_lane_inds.append(good_right_inds)

            if len(good_left_inds) > minpix and np.int64(np.mean(nonzerox[good_left_inds])) < 320:
                leftx_current = np.int64(np.mean(nonzerox[good_left_inds]))
            if len(good_right_inds) > minpix and np.int64(np.mean(nonzerox[good_right_inds])) > 320:
                rightx_current = np.int64(np.mean(nonzerox[good_right_inds]))
        
        leftx = last_window_left / nwindows if 0 <= (last_window_left / nwindows) < 320 else 0
        rightx = last_window_right / nwindows if 640 >= (last_window_right / nwindows) > 320 else 640
        # print('leftx : ', leftx, 'rightx : ', rightx)
        
        left_lane_inds = np.concatenate(left_lane_inds)
        right_lane_inds = np.concatenate(right_lane_inds)

        # leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds]
        # rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds]

        return leftx, lefty, rightx, righty, out_img


def main():
    rospy.init_node('lane_node', anonymous=True)
    Lane = Lane_detection()
    pub = PublishToState()

    cap = cv2.VideoCapture(path) #웹캠으로 받아오기, 2번 사용하면 됨
    # cap = cv2.resize(cap,{500,500})
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,640)  #해상도 조절해주기,웹캠사용시 필요
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT,480)

    while not rospy.is_shutdown():
        while cap.isOpened():
            ret, image = cap.read()

            if not ret:
                print("Can't receive frame (stream end?). Exiting ... ")
                break
            height, width = image.shape[:2]

            vertices = np.array([[(0, 0), (0, height), (width, height), (width, 0)]], dtype=np.int32)
            roi_img = Lane.region_of_interest(image, vertices, (0,0,255))

            mark = np.copy(roi_img)
            # print('mark : ', mark)
            
            mark = Lane.mark_img(roi_img, mark, image)

            gray = cv2.cvtColor(mark, cv2.COLOR_BGR2GRAY)
            leftx, _, rightx, _, out_img = Lane.sliding_window(gray)

            center = (int(0.54 * (leftx + rightx)), 100)
            # print('center : ', center)

            steer = int(0.5 * (leftx + rightx) - 208)

            cv2.circle(image, center, 5, (0, 255, 0), -1)

            time.sleep(0.012)
            cv2.line(image, (width // 2, height), (width // 2, 0), (255, 255, 255), 2)

            cv2.imshow('img', image)
            cv2.imshow('results', mark)
            cv2.imshow('gray',gray)

            # 슬라이딩 윈도우 결과를 출력
            cv2.imshow('sliding_window', out_img)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            steer = np.clip(steer, -22, 22)
            print('steer : ', steer)
            pub.pub_erp(int(steer))

        cap.release()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
