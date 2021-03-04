#!/usr/bin/python
# -*- coding: utf-8 -*-
'''
python image_pixel.py
鼠标左键，查看鼠标位置像素坐标
k： 退出
'''

import sys, os
import cv2
import numpy as np
PATH = os.path.dirname(__file__)
PATH = os.path.abspath(os.path.join(PATH, "../"))


def on_mouse(event ,x,y,flag,param):
    if event is cv2.EVENT_LBUTTONDOWN:
        print("(x: %d,y: %d,):" % (x, y))


def video():
    cap_dev = 4
    cap = cv2.VideoCapture(cap_dev)
    cap.set(5, 30)
    cap.set(3, 1280)
    cap.set(4, 720)
    print('img height :', cap.get(3))
    print('img width:', cap.get(4))
    print('img fps:', cap.get(5))
    while 1:
        ret ,img = cap.read()
        if not ret :break
        cv2.namedWindow('debug', 0)
        cv2.setMouseCallback('debug', on_mouse,)
        cv2.imshow('debug', img)
        k = cv2.waitKey(1)
        if k == ord('k'):
            break

def image():
    image_path = '/home/zjrobot/图片/2020-12-28 18-37-56屏幕截图.png'
    img = cv2.imread(image_path)
    while 1:
        cv2.namedWindow('debug', 0)
        cv2.setMouseCallback('debug', on_mouse)
        cv2.imshow('debug', img)
        k = cv2.waitKey(1)
        if k == ord('k'):
            break

if __name__ == '__main__':
    video()
    # image()
