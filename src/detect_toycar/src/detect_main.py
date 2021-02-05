#!/usr/bin/python3
# -*- coding: utf-8 -*-

import os
import cv2
import rospy
from detect_torch import ToyCar
from camera.camera_model import CameraModel
from create_msgs.msg import toycar_info, toycar_frame
from std_msgs.msg import Header

def main():
    rospy.init_node("detect_toycar")

    detect_param = rospy.get_param("~detect")
    camera_param = rospy.get_param("~camera")
    camera_param_root = rospy.get_param("~camera_param_root")

    detect_interval = rospy.get_param("~detect_interval")

    pub = rospy.Publisher('toycar_info', toycar_info, queue_size=1)

    cam_param_root = os.path.join(camera_param_root,camera_param['far_camera']['path'])
    cam_model = CameraModel(cam_param_root)
    detect = ToyCar(*detect_param)
    cap = cv2.VideoCapture(camera_param['far_camera']['dev'])
    cap.set(5, camera_param['camera_fps'])
    cap.set(3, int(camera_param['image_shape'][0]))
    cap.set(4, int(camera_param['image_shape'][1]))
    print('img height :', cap.get(3))
    print('img width:', cap.get(4))
    print('img fps:', cap.get(5))

    cur_index = 0
    while 1:
        ret, img = cap.read()
        if not ret:
            continue
        # 间隔检测
        cur_index +=1
        if cur_index%detect_interval!=0:
            continue
        box,conf =detect.run(img)
        pos = cam_model.run(box)
        header = Header(cur_index, rospy.Time.now(), 'laser')

        '''发布消息,每一帧,多个物体的检测框,置信度,每个物体两个下边缘点,在激光坐标系的位置'''
        pub_info = [toycar_frame(box=b,confidence=c,position=p) for b,c,p in zip(box,conf,pos)]
        pub.publish(toycar_info(header=header,toycar_info=pub_info))



if __name__ == '__main__':
    main()