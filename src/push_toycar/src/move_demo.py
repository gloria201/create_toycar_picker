#!/usr/bin/python3
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

from camera.camera_model import CameraModel

def detect_aruco(cap,aruco_id = 0):
    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)
    parameters = aruco.DetectorParameters_create()
    while 1:
        ret, img = cap.read()
        if not ret:
            print('Video Capture: No Image')
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY).astype(np.uint8)
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray,
                                                              aruco_dict,
                                                              parameters=parameters)
        if ids is not None and aruco_id in ids:
            # 4*2
            corners = corners[ids[0].tolist().index(aruco_id)].squeeze()
            corners = corners.mean(axis=0)
            print('Find Aruco Point: ',corners)
            return corners
        cv2.imshow(img)
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
    cap.set(3, camera_param['image_shape'][0])
    cap.set(4, camera_param['image_shape'][1])
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
    test()

