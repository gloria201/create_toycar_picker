#!/usr/bin/python3
# -*- coding: utf-8 -*-

'''
激光打印在图像上
'''

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '../'))
sys.path.append(os.path.dirname(__file__))
from src.camera_model import get_camera_model
from Configs import data_collection_config as config,camera_config
from Configs import data_root

import rospy
from sensor_msgs.msg import LaserScan
import cv2
import time
import pickle
import numpy as np
import threading
import queue
import ctypes

if not config['local_ros']:
    os.environ["ROS_HOSTNAME"]=config['ROS_HOSTNAME']
    os.environ["ROS_MASTER_URI"]=config['ROS_MASTER_URI']

if not os.path.exists(data_root):
    os.makedirs(data_root)

class RecordImageLaser():
    def __init__(self):
        self.config = config
        rospy.init_node('laser_listen', anonymous=True)
        self.cap = cv2.VideoCapture(config['cam_id'])
        #self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self.cap.set(3, config['img_wight'])
        self.cap.set(4, config['img_hight'])
        print('img height :', self.cap.get(3))
        print('img width:', self.cap.get(4))
        print('img fps:', self.cap.get(5))
        self.laser_queue = queue.Queue(maxsize=4)
        self.laser_thread = threading.Thread(target=self.laser_listener, args=(self.laser_queue,))
        self.laser_thread.setDaemon(True)
        self.laser_thread.start()
        self.camera_model = get_camera_model(camera_config['camera_model'])(camera_config)
        with open(os.path.join(data_root, 'Laser2Camera.pkl'), 'rb') as f:
            TCL = pickle.load(f)
        self.R_TCL = TCL['R']
        self.T_TCL = TCL['T']

    def run(self):
        theta_range = (-45 / 180 * 3.14, 45 / 180 * 3.14)
        cv2.namedWindow('show', 0)
        while 1:
            ret, image = self.cap.read()
            if ret:
                if self.laser_queue.qsize() > 0:
                    laser = self.laser_queue.get()
                    h, w, _ = image.shape
                    theta = np.arange(len(laser.ranges)) * laser.angle_increment + laser.angle_min
                    ranges = np.array(laser.ranges)
                    x = ranges * np.cos(theta)
                    y = ranges * np.sin(theta)

                    laser_point = np.vstack([x, y, np.zeros(len(x))]).T
                    laser_point = laser_point[np.logical_and(theta < theta_range[1], theta > theta_range[0])]
                    cam_points = self.R_TCL.dot(laser_point.T).T + self.T_TCL
                    pixel_points = self.camera_model.projectPoints(cam_points)
                    pixel_points = pixel_points[
                        np.logical_and(np.logical_and(pixel_points[:, 0] > 0, pixel_points[:, 1] > 0),
                                       np.logical_and(pixel_points[:, 0] < w, pixel_points[:, 1] < h))]
                    #print(len(points))
                    for p in pixel_points:
                        cv2.circle(image, (int(p[0]), int(p[1])), 7, (0, 0, 255), -1)
                else:
                    print('no laser data')
                cv2.imshow('show', image)
                k =cv2.waitKey(1)
                if k == ord('k'):
                    break
        ctypes.pythonapi.PyThreadState_SetAsyncExc(self.laser_thread.ident, ctypes.py_object(SystemExit))
        ctypes.pythonapi.PyThreadState_SetAsyncExc(self.laser_thread.ident, None)

    def laser_callback(self,data,q):
        q.put(data)
        q.get() if q.qsize() > 1 else time.sleep(0.01)

    def laser_listener(self,q):
        
        rospy.Subscriber(self.config['scan_topic_name'], LaserScan, self.laser_callback, callback_args=q)
        rospy.spin()


if __name__ == '__main__':
    app = RecordImageLaser()
    app.run()




