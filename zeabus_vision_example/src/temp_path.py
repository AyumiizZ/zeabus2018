import cv2 as cv
import numpy as np

cap = cv.VideoCapture(0)
img = None


def get_cx(ROI, x, y, w, h):
    sli = 8
    for i in range(sli):
        a = (h*i)/sli
        b = (h*(i+1))/sli
        temp = ROI[a:b, ]
        cv.imshow('temp',temp)
        # cv.waitKey(1000)
        # cnt = cv.findContours(temp, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[1]
        # cnt = max(cnt, key=cv.contourArea)
        # tx, ty, tw, th = cv.boundingRect(cnt)
        # cv.rectangle(frame, (x+tx, y+a), (x+tx+tw, y+a+th), (0, 255, 0), 2)


while True:
    _, frame = cap.read()
    img = frame.copy()  
    himg, wimg = img.shape[:2]
    img = img[:himg/2, ]
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    lower = np.array([17, 92, 94], dtype="uint8")
    upper = np.array([35, 203, 193], dtype="uint8")
    mask = cv.inRange(hsv, lower, upper)
    contours = cv.findContours(
        mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[1]
    if len(contours) >= 1:
        cnt = max(contours, key=cv.contourArea)
        x, y, w, h = cv.boundingRect(cnt)
        ROI = img[y:y+h,x:x+w]
        cv.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
        # cv.imshow("ROI",ROI)
        if(himg/h <= 40): 
            get_cx(ROI, x, y, w, h)
    cv.imshow('img', frame)
    k = cv.waitKey(5) & 0xFF
    if k == 27:
        break

cv.destroyAllWindows()
