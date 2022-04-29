
import cv2
import cv2
from matplotlib import image
import numpy as np
import time
import os

def grey(image):
    return cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
def gauss(image):
    return cv2.GaussianBlur(image, (5, 5), 0)
def canny(image):
    edges = cv2.Canny(image,50,150)
    return edges

def region(image):
    height, width = image.shape
    # triangle = np.array([
    #                    [(100, height), (475, 325), (width, height)]
    #                    ])
    square = np.array([[(0, 300), (160, height // 2), (480, height // 2), (width, 300), (width, height), (0, height)]])
    # trianle = np.array([[(width // 2, 200), ]])
    mask = np.zeros_like(image)
    mask = cv2.fillPoly(mask, square, 255)
    
    mask = cv2.bitwise_and(image, mask)
    return mask

def display_lines(image, lines):
    lines_image = np.zeros_like(image)
    #make sure array isn't empty
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line
            #draw lines on a black image
            cv2.line(lines_image, (x1, y1), (x2, y2), (255, 0, 0), 10)
    return lines_image

def average(image, lines):
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
    left_line = make_points(image, left_avg)
    right_line = make_points(image, right_avg)
    return np.array([left_line, right_line])

def make_points(image, average):
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

def high_contrast(img):
    lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8)) 
    cl = clahe.apply(l)
    limg = cv2.merge((cl, a, b))
    final = cv2.cvtColor(limg, cv2.COLOR_LAB2BGR)
    
    return final


def linearFunction(k, x, x1, y1):
    return k *(x - x1) + y1

def extendLine(pt1, pt2):
    if pt1[0] - pt2[0] != 0:
        dx = pt1[0] - pt2[0]    
        dy = pt1[1] - pt2[1]
        
        k = dy / dx
        
        x = 320
        y = int(linearFunction(k, x, pt1[0], pt1[1]))
    else:
        return pt1[0], pt1[1]
    
    return x, y

    


def divideLine(img, copy):
    
    left = []
    right = []
    h, w = img.shape
    
    for y in range(h):
        for x in range(w):
            if img[y][x] == 255:
                if y > 150:
                    if x < 100:
                        left.append((x, y))
                    if x > 540:
                        right.append((x, y))
    
    if len(right) != 0 or len(left) != 0:
        for i in range(len(right)):
            cv2.circle(copy, right[i], 2, (0, 0, 255), -1)

        for i in range(len(left)):
            cv2.circle(copy, left[i], 2, (0, 0, 255), -1)   

        if len(right) > 2:
            LX = left[:][0]
            LY = left[:][1]
        else:
            LX = [0]
            LY = [300]
            
        if len(right) > 2:
            RX = right[:][0]
            RY = right[:][1]
        else:
            RX = [640]
            RY = [300]
        
        avgLX = int(np.mean(LX, axis=0))
        avgLY = int(np.mean(LY, axis=0))
        avgRX = int(np.mean(RX, axis=0))
        avgRY = int(np.mean(RY, axis=0))
        
        # il, ir = len(left), len(right)
        # center = ((left[il // 2][0] + right[ir // 2][0]) // 2, (left[il // 2][1] + right[ir // 2][1]) // 2)
        
        center = ((avgLX + avgRX) // 2, (avgLY + avgRY) // 2)
        print("center :", center)
        cv2.line(copy, (w // 2, h), (w // 2, 0), (255, 255, 255), 5)
        cv2.circle(copy, center, 10, (0, 255, 0), -1)
    # if (len(right) < 2): 
    #     print("Turn right!!!!")
    # else:
    #     print("left :", left[0], left[-1])
    #     print("right :", right[0], right[-1])
    #     x, y = extendLine(right[0], right[-1])
    #     cv2.circle(copy, right[0])
    #     cv2.line(copy, right[0], right[-1], (0, 0, 255), 5)
        
    # if (len(left) < 2): 
    #     print("Turn left!!!!")
    # else:
    #     print("left :", left[0], left[-1])
    #     print("right :", right[0], right[-1])
    #     x, y = extendLine(left[0], left[-1])
    #     cv2.line(copy, left[0], left[-1], (0, 0, 255), 5)
    
    
    return img
    

cap = cv2.VideoCapture(os.getcwd() + "/lane_detection/origin_code(C version)/sample_data/track-s.mkv")
fourcc = cv2.VideoWriter_fourcc(*'X264')
out = cv2.VideoWriter('output.avi', fourcc, 20.0, (640,480))



while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        print("Can't receive frame (stream end?). Exiting ... ")
        break
    
    h, w, _= frame.shape
    
    # print("shape :", frame.shape)
    # frame = high_contrast(frame)
    copy = frame
    gaus = gauss(frame)
    edges = cv2.Canny(gaus,50,150)
    # edges = region(edges)
    edges = region(edges)
    # isolated = region(frame)
    # # isolated = cv2.cvtColor(isolated, cv2.COLOR_BGR2HSV)
    # # lines = cv2.HoughLinesP(isolated, 2, np.pi/180, 100, np.array([]), minLineLength=40, maxLineGap=5)
    # lines = cv2.HoughLinesP(edges, 1, np.pi/180, 60, 60, 20)
    # averaged_lines = average(copy, lines)
    # black_lines = display_lines(copy, averaged_lines)
    # lanes = cv2.addWeighted(copy, 0.8, black_lines, 1, 1)
    # frame = canny(frame)
    
    divideLine(edges, copy)
    cv2.imshow('edges', edges)
    cv2.imshow('copy', copy)
    # cv2.line(frame, (100, 500), (400, 100), (0, 0, 255), 5)
    # cv2.imshow('frame', lanes)
    # time.sleep(1)
    time.sleep(0.05)
    if cv2.waitKey(1) == ord('q'):
        break
        

cap.release()
out.release()
cv2.destroyAllWindows()