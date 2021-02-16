#!/home/gloria/anaconda3/envs/py3_8_env/bin/python
# -*- coding: utf-8 -*-

import os
import cv2
import rospy
from detect_torch import ToyCar
from camera.camera_model import CameraModel
#from create_msgs.msg import toycar_info, toycar_frame
from std_msgs.msg import Header
from visualization_msgs.msg import Marker

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
    while not rospy.is_shutdown():
        ret, img = cap.read()
        if not ret:
            continue
        # 间隔检测
        cur_index +=1
        if cur_index%detect_interval!=0:
            continue
        box,conf =detect.run(img)
        points = [[[b[0],b[3]],[b[2],b[3]]] for b in box]

        for b in box:
            cv2.rectangle(img, (int(b[0]), int(b[1])), (int(b[2]), int(b[3])), (0, 254, 0), 1)
        cv2.imshow('debug', img)
        cv2.waitKey(1)

        if len(box) == 0:
            pos = []
        else:
            pos = cam_model.run(points)
        header = Header(cur_index, rospy.Time.now(), 'laser')

        '''发布消息,每一帧,多个物体的检测框,置信度,每个物体两个下边缘点,在激光坐标系的位置'''
        pub_info = [toycar_frame(box=b,confidence=c,position=p) for b,c,p in zip(box,conf,pos)]
        pub.publish(toycar_info(header=header,toycar_info=pub_info))

def test():
    rospy.init_node("detect_toycar")

    detect_param = rospy.get_param("~detect")
    camera_param = rospy.get_param("~camera")
    camera_param_root = rospy.get_param("~camera_param_root")

    detect_interval = rospy.get_param("~detect_interval")

    marker_pub = rospy.Publisher("/cube", Marker, queue_size=10)

    cam_param_root = os.path.join(camera_param_root, camera_param['far_camera']['path'])
    cam_model = CameraModel(cam_param_root)
    detect = ToyCar(**detect_param)
    cap = cv2.VideoCapture(camera_param['far_camera']['dev'])
    cap.set(5, camera_param['camera_fps'])
    cap.set(3, int(camera_param['image_shape'][0]))
    cap.set(4, int(camera_param['image_shape'][1]))
    print('img height :', cap.get(3))
    print('img width:', cap.get(4))
    print('img fps:', cap.get(5))

    cur_index = 0
    while not rospy.is_shutdown():
        ret, img = cap.read()
        if not ret:
            continue
        # 间隔检测
        cur_index += 1
        if cur_index % detect_interval != 0:
            continue
        box, conf = detect.run(img)
        for b in box:
            cv2.rectangle(img, (int(b[0]), int(b[1])), (int(b[2]), int(b[3])), (0, 254, 0), 1)
        cv2.imshow('debug', img)
        cv2.waitKey(1)

        if len(box) ==0 :
            continue
        points = [[[(b[0]+b[2])/2, b[3]]] for b in box]
        pos = cam_model.run(points)
        for i,p in enumerate(pos):
            marker = mark(p[0],ids=i)
            marker_pub.publish(marker)

def mark(pos,ids=0):
    marker = Marker()

    # 指定Marker的参考框架
    marker.header.frame_id = "/map"
    # 时间戳
    marker.header.stamp = rospy.Time.now()

    # ns代表namespace，命名空间可以避免重复名字引起的错误
    marker.ns = "basic_shapes"

    # Marker的id号
    marker.id = ids

    # Marker的类型，有ARROW，CUBE等
    marker.type = Marker.CYLINDER

    # Marker的尺寸，单位是m
    marker.scale.x = 0.05
    marker.scale.y = 0.05
    marker.scale.z = 0.2

    # Marker的动作类型有ADD，DELETE等
    marker.action = Marker.ADD

    # Marker的位置姿态
    if len(pos)==0:
        marker.pose.position.x = 0
        marker.pose.position.y = 1
    else:
        marker.pose.position.x = pos[0]
        marker.pose.position.y = pos[1]
    marker.pose.position.z = 0.1
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

    # Marker的颜色和透明度
    marker.color.r = 0.0
    marker.color.g = 0.8
    marker.color.b = 0.0
    marker.color.a = 0.5

    # Marker被自动销毁之前的存活时间，rospy.Duration()意味着在程序结束之前一直存在
    marker.lifetime = rospy.Duration()
    return marker

if __name__ == '__main__':
    # main()
    test()
