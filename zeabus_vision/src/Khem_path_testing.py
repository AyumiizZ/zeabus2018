#!/usr/bin/python
import rospy
import numpy as np
import rosbag
import cv2 as cv
img = None
bag = rosbag.Bag('01.bag', "r")
messages = bag.read_messages(topics="/bottom/left/image_raw/compressed")
num_image = bag.get_message_count(topic_filters=["/bottom/left/image_raw/compressed"])
def main() :
    global img
    for i in range(num_image) :
        # print ('Hello')
        topic,msg,t = messages.next()
        img = np.fromstring(msg.data , dtype=np.uint8)
        img = cv.resize(cv.imdecode(img,1),(0,0),fx=1 , fy=1)
        size = 500
        r = 1.0*size / img.shape[1]
        dim = (size, int(img.shape[0] * r))
        resized = cv.resize(img, dim, interpolation = cv.INTER_AREA)
        img = resized
        hsv = cv.cvtColor(img,cv.COLOR_BGR2HSV)
        #cv.imshow('1_gray',hsv)
        h , s ,v = cv.split(img)
        s.fill(100)
        # v.fill(200)
        gray = cv.merge((h,s,v))
        #cv.imshow('2_gray',gray)
        gray = cv.cvtColor(img,cv.COLOR_HSV2BGR)
        gray = cv.cvtColor(img,cv.COLOR_BGR2GRAY)
        #cv.imshow('3_gray',gray)
        # h , s ,v = cv.split(img)
        # print np.mean(s)
        cv.imshow('g',gray)
        # s.fill(150)
        # gray = cv.merge((h,s,v))
        gray[gray > 240] = 0
        gray[gray <= 140] = 0
        # gray[gray > 340- np.mean(s)] = 0
        # gray[gray <= 200 - np.mean(s)] = 0
        cv.imshow('gray',gray)
        cnt = cv.findContours(gray, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[1]
        cv.drawContours(img,cnt,-1,(0,0,0),3)
        rectangle = False
        if len(cnt) != 0:
            cnt = (sorted(cnt,key=cv.contourArea,reverse=True)[0])
            rect = cv.minAreaRect(cnt)
            area = cv.contourArea(cnt)
            for c in cnt :
                peri = cv.arcLength(c, True)
                approx = cv.approxPolyDP(c, 0.04 * peri, True)
                if len(approx) == 4:
                    rectangle = True
            # if area > 2000 and rectangle: 
            # rect = cv.minAreaRect(i)
            if area > 2000 :
                box = cv.boxPoints(rect)
                box = np.int0(box)
                free = img.copy()
                cv.drawContours(free,[box],0,(0,255,255),2)
                M = cv.moments(cnt)
                ROI_cx = int(M["m10"]/M["m00"])
                ROI_cy = int(M["m01"]/M["m00"]) 
                cv.circle(free,(ROI_cx,ROI_cy),3,(255,0,0),-1)
                cv.imshow('box',free)
        #     cnt = sorted(cnt, key = cv.contourArea, reverse = True)[0]
        #     print cnt
        # cv.drawContours(gray,cnt,-1,(170,255,0),3)
        # for i in cnt :
        #     if cv.contourArea(i) == (max(cnt,key=cv.contourArea).all():
        # rect = cv.minAreaRect(max(cnt,key=cv.contourArea))
        # box = cv.boxPoints(rect)
        # box = np.int0(box)
        # cv.drawContours(img,[box],0,(255,255,255),2)
        # cv.imshow('box',img)
        k = cv.waitKey(1) & 0xFF
        if k == 27:
            break
        # cv.imshow('res'',res)
        # cv.imshow('mask,mask)
    cv.destroyAllWindows()
if __name__ == "__main__" :
    main()        
