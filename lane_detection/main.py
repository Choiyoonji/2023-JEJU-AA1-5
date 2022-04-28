from copy import deepcopy
from unittest import result
import cv2
from cv2 import getPerspectiveTransform
from cv2 import warpPerspective
from cv2 import imread
from numpy import mat
import os

IMG_WIDTH = 640
IMG_Height = 480
PERSPECTIVE_IMG_W = 640
PERSPECTIVE_IMG_H = 480

ASSIST_BASE_LINE = 130 
ASSIST_BASE_WIDTH = 30 

src = [(0, 0), (-290, 330), (640, 0), (640 + 290, 330)]
dest = [(0, 0), (0, PERSPECTIVE_IMG_H), (PERSPECTIVE_IMG_W, 0), (PERSPECTIVE_IMG_W, PERSPECTIVE_IMG_H)]

def Perspecitve(img):
    matrix = getPerspectiveTransform(src, dest)
    result = warpPerspective(img, matrix, (PERSPECTIVE_IMG_W, PERSPECTIVE_IMG_H))
    
    return result

def Canny_Edge_Detection(img):
    mat_blur_img = cv2.blur(img , (3, 3))
    mat_canny_img = cv2.Canny(mat_blur_img, 70, 170, 3)
    
    return mat_canny_img

img_width = 640
img_height = 480

GREEN = (0, 255, 0)
RED = (0, 0, 255)
BLUE = (255, 0, 0)
YELLOW = (0, 255, 255)

# print(os.getcwd())

mat_image_org_color = cv2.imread(os.getcwd() + "2020-08-11-211923.jpg")
cv2.copyTo(mat_image_org_color, mat_image_org_color_overlay)

img_width, img_height = mat_image_org_color.shape 

print("Image size[%{}, %{}]", img_width, img_height)



print("H")










