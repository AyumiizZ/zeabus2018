import cv2 as cv
import statistics
import numpy as np
import time
import os
# from scipy import stats

def get_mode1(data):
    data = np.array(data)
    if len(data.shape) > 1:
        data = data.ravel()
    try:
        mode = statistics.mode(data)
    except ValueError:
        mode = None
    return mode

def get_mode2(data):
    data = data.ravel()
    count = np.bincount(data)
    max = 0 
    mode = 127
    for i in range(len(count)):
        if count[i] > max:
            max = count[i]
            mode = i
    return mode

def get_mode3(data):
    data = data.ravel()
    count = np.bincount(data)
    max = count.max()
    count = list(count)
    mode = count.index(max)
    return mode

def get_mode4(data):
    data = data.ravel()
    count = np.bincount(data)
    max = count.max()
    mode = np.where(count == max)
    return np.mean(mode)

def get_mode5(data):
    data = data.ravel()
    count = np.bincount(data)
    max = 0 
    mode = 127
    length = len(count) -1
    end = int(length/2) + 1
    for i in range(end):
        if count[i] > max:
            max = count[i]
            mode = i
        if count[length - i] > max:
            max = count[length - i]
            mode = i
    return mode

# def get_mode5(data):
#     data = data.ravel()
    
#     return stats.mode(data)

if __name__ == '__main__':
    # print(__file__) 
    dir = os.path.dirname(os.path.realpath(__file__))
    img = cv.imread(dir+'/test_mode.jpg',0)

    print('get mode 1')
    start = time.time()
    print(get_mode1(img))
    end = time.time()
    print('time:',end-start, 'sec')

    print('get mode 2')
    start = time.time()
    print(get_mode2(img))
    end = time.time()
    print('time:',end-start, 'sec')

    print('get mode 3')
    start = time.time()
    print(get_mode3(img))
    end = time.time()
    print('time:',end-start, 'sec')

    print('get mode 4')
    start = time.time()
    print(get_mode4(img))
    end = time.time()
    print('time:',end-start, 'sec')
    
    print('get mode 5')
    start = time.time()
    print(get_mode5(img))
    end = time.time()
    print('time:',end-start, 'sec')