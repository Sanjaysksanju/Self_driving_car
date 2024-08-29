import cv2
import numpy as np
import math

def detect_lane(frame):
    def slope(vx1, vx2, vy1, vy2):
        m = float(vy2 - vy1) / float(vx2 - vx1)
        theta1 = math.atan(m)
        return theta1 * (180 / np.pi)

    img = cv2.resize(frame, (600, 600))
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    equ = cv2.equalizeHist(gray)
    blur = cv2.GaussianBlur(equ, (5, 5), 0)
    ret, thresh = cv2.threshold(blur, 240, 255, cv2.THRESH_BINARY)
    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(thresh, contours, -1, (255, 0, 0), 3)
    lines = cv2.HoughLinesP(thresh, cv2.HOUGH_PROBABILISTIC, np.pi/180, 25, minLineLength=10, maxLineGap=40)
    
    l = r = 0
    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                if round(x2 - x1) != 0:
                    arctan = slope(x1, x2, y1, y2)
                    if 250 < y1 < 600 and 250 < y2 < 600:
                        if round(arctan) >= -80 and round(arctan) <= -30:
                            r += 1
                        elif round(arctan) >= 30 and round(arctan) <= 80:
                            l += 1
    
    if l >= 10:
        return 'left'
    elif r >= 10:
        return 'right'
    else:
        return 'straight'
