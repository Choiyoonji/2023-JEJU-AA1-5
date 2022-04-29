import os
from cv2 import namedWindow
import numpy as np

from copy import deepcopy
from unittest import result
import cv2
from cv2 import getPerspectiveTransform
from cv2 import warpPerspective
from cv2 import imread
from numpy import imag, mat


IMG_WIDTH = 640
IMG_Height = 480
PERSPECTIVE_IMG_W = 640
PERSPECTIVE_IMG_H = 480

ASSIST_BASE_LINE = 130 
ASSIST_BASE_WIDTH = 30 

# src = np.array([[0, 0], [-290, 330], [640, 0], [640 + 290, 330]])
# dest = np.array([[0, 0], [0, PERSPECTIVE_IMG_H], [PERSPECTIVE_IMG_W, 0], [PERSPECTIVE_IMG_W, PERSPECTIVE_IMG_H]])

def Perspective(img):
    src = np.float32([[0, 0], [-290, 330], [640, 0], [640 + 290, 330]])
    dest = np.float32([[0, 0], [0, 480], [640, 0], [640, 480]])

    matrix = cv2.getPerspectiveTransform(src, dest)
    result = cv2.warpPerspective(img, matrix, (PERSPECTIVE_IMG_W, PERSPECTIVE_IMG_H))
    
    return result

def Canny_Edge_Detection(img):
    mat_blur_img = cv2.GaussianBlur(img, (3, 3), 0)
    mat_canny_img = cv2.Canny(mat_blur_img, 70, 170)
    
    return mat_canny_img

def findLines(img):
    img_original = img.copy()
    # gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    edges = cv2.Canny(img, 50, 150, apertureSize=3)

    lines = cv2.HoughLines(edges,1,np.pi/180,100)

    for i in range(lines.shape[0]):
        left_side = []
        right_side = []
        
        # 직진 여부 확인

            
        for rho, theta in lines[i]:
            # 중요한 부분 : theta값에 따라 steer값 조절 더 해야함
            if 1.39626 < theta < 1.91986: # 노이즈 제거
                continue
            elif 2 < theta < 2.5:
                print("Turn steer to left")
            elif 0.53 < theta < 1.2:
                print("Turn steer to right")
            elif 0.49 < theta < 0.53:
                print("###########")
                print("###########")
                print("Go straight")
                print("###########")
                print("###########")
            else:
                continue
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a*rho
            y0 = b*rho
            x1 = int(x0 + 1000*(-b))
            y1 = int(y0+1000*(a))
            x2 = int(x0 - 1000*(-b))
            y2 = int(y0 -1000*(a))
            
            print(rho, theta)
            print(x0, y0, x1, y1, x2, y2)
            
            cv2.circle(img, ((x1 + x2) // 2, (y1 + y2) // 2), 5, (0, 0, 255))
            cv2.line(img,(x1,y1),(x2,y2),(0,0,255),2)
            
        for _, theta in lines[i]:
            if 0.45 < theta < 0.53:
                left_side.append(theta)
            elif 2.68 < theta < 2.82:
                right_side.append(theta)
            
        if len(left_side) > 4 and len(right_side) > 4:
            print("###########")
            print("###########")
            print("Go straight222222222222")
            print("###########")
            print("###########")
            continue
    # res = np.vstack((img_original,img))
    cv2.imshow('img', img)

def drawLines(img):
    
    guide_width1 = 50
    guide_height1 = 20
    guide_l_center = 0 + 70
    guide_r_center = img_width - 70
    
    cv2.rectangle(img, (50, ASSIST_BASE_LINE - ASSIST_BASE_WIDTH), (img_width - 50 , ASSIST_BASE_LINE + ASSIST_BASE_WIDTH), (0,255,0), 1, cv2.LINE_AA)
    cv2.line(img, (guide_l_center - guide_width1, ASSIST_BASE_LINE), (guide_l_center, ASSIST_BASE_LINE), (0,255,255), 1, 0)
    cv2.line(img, (guide_l_center, ASSIST_BASE_LINE), (guide_l_center + guide_width1, ASSIST_BASE_LINE), (0,255,255), 1, 0)
    
    cv2.line(img, (guide_l_center - guide_width1, ASSIST_BASE_LINE - guide_height1), (guide_l_center - guide_width1, ASSIST_BASE_LINE + guide_height1), (0,255,255), 1, 0)
    cv2.line(img, (guide_l_center + guide_width1, ASSIST_BASE_LINE - guide_height1), (guide_l_center + guide_width1, ASSIST_BASE_LINE + guide_height1), (0,255,255), 1, 0)
    cv2.line(img, (guide_r_center - guide_width1, ASSIST_BASE_LINE - guide_height1), (guide_r_center - guide_width1, ASSIST_BASE_LINE + guide_height1), (0,255,255), 1, 0)
    cv2.line(img, (guide_r_center + guide_width1, ASSIST_BASE_LINE - guide_height1), (guide_r_center + guide_width1, ASSIST_BASE_LINE + guide_height1), (0,255,255), 1, 0)
    
    cv2.line(img, (int(img_width // 2), int(ASSIST_BASE_LINE - guide_height1 * 1.5)) , (int(img_width // 2), int(ASSIST_BASE_LINE + guide_height1 * 1.5)), (255,255,255), 2, 0)
    
    cv2.line(img, (guide_r_center - guide_width1, ASSIST_BASE_LINE), (guide_r_center, ASSIST_BASE_LINE), (0,255,255), 1, 0)
    cv2.line(img, (guide_r_center, ASSIST_BASE_LINE), (guide_r_center + guide_width1, ASSIST_BASE_LINE), (0,255,255), 1, 0)

img_width = 640
img_height = 480

GREEN = (0, 255, 0)
RED = (0, 0, 255)
BLUE = (255, 0, 0)
YELLOW = (0, 255, 255)

# 경로는 환경에 따라 print(os.getcwd()) 실행 후 재설정
# print(os.getcwd())
image = cv2.imread(os.getcwd() + "/lane_detection/sample_image/2020-08-11-211923.jpg")
image_color_overlayed = image
# image_grayscale_overlayed = np.zeros((img_width, img_height, 3), np.uint8)

# cv2.copyTo(src=image, dst=image_overlayed)

# cv2.imshow("image", image)
# cv2.waitKey(0)

print(image.shape)

img_width, img_height, _ = image.shape 

print("Image size[{}, {}]".format(img_width, img_height))

# cv2.namedWindow("Display window", cv2.WINDOW_NORMAL)
# cv2.resizeWindow("Display window", img_width, img_height)
# cv2.moveWindow("Display window", 10, 20)

# cv2.waitKey(0)
# cv2.namedWindow("Gray Image window", cv2.WINDOW_NORMAL)
# cv2.resizeWindow("Gray Image window", img_width, img_height)
# cv2.moveWindow("Gray Image window", 700, 20)

# cv2.namedWindow("Canny Edge Image window", cv2.WINDOW_NORMAL)
# cv2.resizeWindow("Canny Edge Image window", img_width, img_height)
# cv2.moveWindow("Canny Edge Image window", 10, 520)

# cv2.namedWindow("Display window", cv2.WINDOW_NORMAL)
# cv2.resizeWindow("Display window", img_width, img_height)
# cv2.moveWindow("Display window", 10, 20)
# cv2.namedWindow("Gray Image window", cv2.WINDOW_NORMAL)
# cv2.resizeWindow("Gray Image window", img_width, img_height)
# cv2.moveWindow("Gray Image window", 700, 20)

count = 0
images = os.listdir(os.getcwd() + "/lane_detection/sample_image")
print(images)

while True:
    index = count % 7

    # image = cv2.imread(os.getcwd() + "/lane_detection/sample_image/2020-08-11-211923.jpg")
    image = cv2.imread(os.getcwd() + "/lane_detection/sample_image/" + images[index])
    print()
    print()
    print(images[index])
    image_color_overlayed = image
    image_grayscale_overlayed = cv2.cvtColor(image_color_overlayed, cv2.COLOR_RGB2GRAY)

    image_canny_edge = Canny_Edge_Detection(image_grayscale_overlayed)
    img = Perspective(image_canny_edge)
    # cv2.imshow("Show", image_canny_edge)
    linesP = cv2.HoughLinesP(img, 1, np.pi/180, 60, 60, 20)

    print("Line Number : {}".format(linesP.shape))
    
    findLines(image_grayscale_overlayed)
    # if linesP is not None: # 라인 정보를 받았으면
    #     for i in range(linesP.shape[0]):
    #         pt1 = (linesP[i][0][0], linesP[i][0][1]) # 시작점 좌표 x,y
    #         pt2 = (linesP[i][0][2], linesP[i][0][3]) # 끝점 좌표, 가운데는 무조건 0
    #         cv2.line(image, pt1, pt2, (0, 0, 255), 1, cv2.LINE_AA)
    # lines = cv2.HoughLines(img, 1, np.pi / 180, 150, None, 0, 0)           

    # cv2.imshow("Source", image)
    # cv2.imshow("Detected Lines (in red) - Standard Hough Line Transform", image_canny_edge)
    # cv2.imshow("Detected Lines (in red) - Probabilistic Line Transform", cdstP)
    
    # drawLines(image_color_overlayed)
    # drawLines(image_grayscale_overlayed)
    
	  
    # cv2.imshow("Display window", image_color_overlayed)
	# cv2.imshow("Display window", mat_image_org_color)
    # cv2.imshow("Gray Image window", image_grayscale_overlayed) # mat_image_org_gray
    
    count += 1
    if cv2.waitKey(0) & 0xFF == 27:
        break
    # break

cv2.destroyAllWindows()