'''
    File name: get_features.py
    Author: zeabus2018
    Date created: 2018/05/24
    Python Version: 3.6.1
'''

import cv2 as cv
import math
from operator import itemgetter
from sklearn import linear_model
import csv

mk = []
combination_result = []
permutation_result = []



def combination(n, r, k, res, ct):
    global combination_result
    if ct > 0:
        res += ' ' + str(k)
    if ct == r:
        combination_result.append(res)
    else:
        for i in range(k + 1, n + 1):
            combination(n, r, i, res, ct + 1)


def permutation(n, r, k, res, ct):
    global permutation_result, mk
    if ct > 0:
        res += ' ' + str(k)
    if ct == r:
        permutation_result.append(res)
    else:
        for i in range(1, n + 1):
            if not i in mk:
                mk.append(i)
                permutation(n, r, i, res, ct + 1)
                mk.remove(i)






def get_index(n, r):
    global combination_result
    combination_result = []
    mk = []
    combination(n, r, 0, '', 0)
    combination_result = list(set(combination_result))

    res = []
    for i in combination_result:
        tmp = [int(k) for k in i.split(' ')[1:]]
        res.append(tmp)

    return sorted(res)


def generate_data_training():
    circles = []
    for i in [2, 5, 6]:
        prefix = 'dice-' + str(i) + '-'
        for j in range(44):
            img = cv.imread(img_path + prefix + str(j) + '.jpg')
            if img is None:
                break
            img = cv.resize(img,(60,60))
            circle = get_point(img)
            circle['y'] = i
            circles.append(circle)

    fcsv = abs_path + 'dataset.csv'
    with open(fcsv, 'w', newline='\n') as csvfile:
        fieldnames = ['x1', 'x2', 'x3', 'x4', 'x5', 'x6', 'x7', 'x8', 'x9', 'y']
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()

        for c in circles:
           
            writer.writerow(c)


def generate_images_training(img, point=5, ct=0):
    prefix = 'dice-' + str(point) + '-'
    circles = get_point(img)
    list_of_index = get_index(point, 0) + \
        get_index(point, 1) + get_index(point, 2)
    ct = ct
    for index in list_of_index:
        img_save = img.copy()
        for i in index:
            x, y, radius = circles[i - 1]
            cv.circle(img_save, (x, y), radius + 2, (0, 0, 0), -1)
        cv.imwrite(img_path + prefix + str(ct) + '.jpg', img_save)
        ct += 1


def generate_images_training_all():
    img = cv.imread(img_path + 'dice_5_0.jpg', 1)
    generate_images_training(img, 5)
    img = cv.imread(img_path + 'dice_6_0.jpg', 1)
    generate_images_training(img, 6)
    img = cv.imread(img_path + 'dice_6_1.jpg', 1)
    generate_images_training(img, 6, 22)


def main():
    # generate_images_training_all()
    generate_data_training()


if __name__ == '__main__':
    main()
