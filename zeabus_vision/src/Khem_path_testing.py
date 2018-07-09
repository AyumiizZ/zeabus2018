#!/usr/bin/python
import rospy
import numpy as np
import rosbag
import cv2 as cv
from scipy import stats
from statistics import mode
img = None
bag = rosbag.Bag('07.bag', "r")
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
        cv.imshow('first',img)
        hsv = cv.cvtColor(img,cv.COLOR_BGR2HSV)
        # cv.imshow('1_gray',hsv)
        h , s ,v = cv.split(hsv)
        # print s
        # v.fill(np.mean(s))
        dark = np.mean(v,dtype='uint64')
        # print dark
        # print '---'
        v = np.array(v) - dark
        # print s
        # print '---'
        # print s - np.mean(s)
        # # dark.astype(np.uint8)
        # s.fill(s)
        gray = cv.merge((h,s,v))
        # h,s,v = cv.split(gray)
        # print s
        # cv.imshow('2_gray',gray)
        gray = cv.cvtColor(gray,cv.COLOR_HSV2BGR)    
        cv.imshow('second',gray)
        gray = cv.cvtColor(gray,cv.COLOR_BGR2GRAY)
        cv.imshow('third',gray)
        # a = np.array(s)
        # print(" ----------------- I will print a -------------------")
        # print a
        # m = stats.mode(a)[0]
        # print(" ----------------- I will print m -------------------")
        # print m
        # p = stats.mode(m)[0]
        # for i in m :
        # q = mode(m[0])
        # print(" ----------------- I will print q -------------------")
        # print q
        # cv.imshow('3_gray',gray)
        # h , s ,v = cv.split(img)
        # print s
        # print np.mean(v)
        # md = []
        # mod = []
        # for c in s :
        #     for i in stats.mode(c) :
        #         md.append(i)
        #         # print md[0][0][0]
        #         mod.append(stats.mode(md)[0][0][0])
        # print mode(mod)               
        # cv.imshow('g',gray)
        # s.fill(150)
        # gray = cv.merge((h,s,v))
        # gray[gray > np.mean(v)-30] = 0
        # gray[gray <= np.mean(v)-140] = 0
        gray[gray > 169] = 0
        gray[gray <= 77] = 0
        cv.imshow('gray',gray)
        cnt = cv.findContours(gray, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[1]
        cv.drawContours(img,cnt,-1,(0,0,0),3)
        rectangle = False
        if len(cnt) != 0:
            cnt = (sorted(cnt,key=cv.contourArea,reverse=True)[0])
            rect = cv.minAreaRect(cnt)
            area = cv.contourArea(cnt)
            # for c in cnt :
            #     peri = cv.arcLength(c, True)
            #     approx = cv.approxPolyDP(c, 0.04 * peri, True)
            #     if len(approx) == 4:
            #         rectangle = True
            # if area > 2000 and rectangle: 
            # rect = cv.minAreaRect(i)
            if area > 1800 :
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
