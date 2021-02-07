#!/usr/bin/python
# -*- coding: utf-8 -*-
'''
demo: 实现二维码的作为目标,利用movebase导航过去
'''

'''
step 1: 检测二维码
step 2: 计算二维码的位置
step 3: 发送move base
    成功后继续
'''

import os
import cv2
# import tf
import cv2.aruco as aruco
import numpy as np
import rospy
import actionlib
from std_msgs.msg import Header
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion
from visualization_msgs.msg import Marker

from camera.camera_model import CameraModel

def detect_aruco(cap,aruco_id = 0):
    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)
    parameters = aruco.DetectorParameters_create()

    cv2.namedWindow('FIND ARUCO', 0)
    while 1:
        ret, img = cap.read()
        if not ret:
            print('Video Capture: No Image')
            break
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY).astype(np.uint8)
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray,
                                                              aruco_dict,
                                                              parameters=parameters)
        if ids is not None and aruco_id in ids:
            # 4*2
            corners = corners[ids[0].tolist().index(aruco_id)].squeeze()
            corners = corners.mean(axis=0)
            cv2.imshow('FIND ARUCO',img)
            cv2.waitKey(1)
            print('Find Aruco Point: ',corners)
            return corners
        cv2.imshow('FIND ARUCO',img)
        cv2.waitKey(1)


def move(move_base,goal):
    move_base.send_goal(goal)
    finished_within_time = move_base.wait_for_result(rospy.Duration(10))
    state = move_base.get_state()
    print(state)

    # If we don't get there in time, abort the goal
    # 如果一分钟之内没有到达，放弃目标
    if not finished_within_time:
        move_base.cancel_goal()
        rospy.loginfo("Timed out achieving goal")
    else:
        # We made it!
        state = move_base.get_state()
        print(state)
        if state == GoalStatus.SUCCEEDED:
            rospy.loginfo("Goal succeeded!")
    move_base.cancel_goal()

def main():
    '''config'''
    rospy.init_node("move_demo")

    camera_param = rospy.get_param("~camera")
    camera_param_root = rospy.get_param("~camera_param_root")
    cam_param_root = os.path.join(camera_param_root,camera_param['far_camera']['path'])

    # 假定激光和机器人中心位置一致
    # listener = tf.TransformListener()
    # (trans, rot) = listener.lookupTransform('laser', 'basefoot_print', rospy.Time(0))

    # pub = rospy.Publisher('toycar_info', toycar_info, queue_size=1)
    cam_model = CameraModel(cam_param_root)
    cap = cv2.VideoCapture(camera_param['far_camera']['dev'])
    cap.set(5, camera_param['camera_fps'])
    cap.set(3, int(camera_param['image_shape'][0]))
    cap.set(4, int(camera_param['image_shape'][1]))
    print('img height :', cap.get(3))
    print('img width:', cap.get(4))
    print('img fps:', cap.get(5))


    move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

    while 1:
        point = detect_aruco(cap)
        # pixel to laser
        laser_p = cam_model.cam_world2laser([point])[0]
        # laser to map

        # goal
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'laser'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point([laser_p[0],laser_p[1], 0]),Quaternion([0,0,0,1]))
        # move
        move(move_base,goal)
        rospy.loginfo('************ Finish ************')

def mark(pos):
    marker = Marker()

    # 指定Marker的参考框架
    marker.header.frame_id = "/map"
    # 时间戳
    marker.header.stamp = rospy.Time.now()

    # ns代表namespace，命名空间可以避免重复名字引起的错误
    marker.ns = "basic_shapes"

    # Marker的id号
    marker.id = 0

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


def test_aruco_location():
    import time
    import random
    '''config'''
    rospy.init_node("move_demo")

    camera_param = rospy.get_param("~camera")
    camera_param_root = rospy.get_param("~camera_param_root")
    cam_param_root = os.path.join(camera_param_root, camera_param['far_camera']['path'])

    cam_model = CameraModel(cam_param_root)

    cap = cv2.VideoCapture(camera_param['far_camera']['dev'])
    cap.set(5, camera_param['camera_fps'])

    cap.set(3, int(camera_param['image_shape'][0]))
    cap.set(4, int(camera_param['image_shape'][1]))
    print('img height :', cap.get(3))
    print('img width:', cap.get(4))
    print('img fps:', cap.get(5))

    marker_pub = rospy.Publisher("/cube", Marker, queue_size=10)
    while not rospy.is_shutdown() :
        point = detect_aruco(cap)
        # pixel to laser
        laser_p = cam_model.run([[point]])[0][0]
        print(laser_p)
        # laser to map
        marker = mark(laser_p)
        marker_pub.publish(marker)
        time.sleep(0.3)
        #
        rospy.loginfo('************ Finish ************')

def test():
    import sys
    assert len(sys.argv)==4
    laser_p = [float(x) for x in sys.argv[1:3]]
    cur_index = int(sys.argv[3])
    rospy.init_node("test_move_demo")
    move_base = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
    move_base.wait_for_server(rospy.Duration(5))
    # goal
    goal = MoveBaseGoal()
    header = Header(cur_index, rospy.Time.now(), 'map')
    goal.target_pose.header = header
    goal.target_pose.pose = Pose(Point(laser_p[0], laser_p[1], 0), Quaternion(0, 0, 0, 1))
    # move
    print(goal)
    move(move_base, goal)
    rospy.loginfo('************ Finish ************')

if __name__ == '__main__':
    # main()
    test_aruco_location()

