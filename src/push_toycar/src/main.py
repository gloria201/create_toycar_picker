#!/home/hushunda/anaconda3/envs/py36/bin/python

#/home/gloria/anaconda3/envs/py3_8_env/bin/python
# -*- coding: utf-8 -*-

import os
import cv2
import rospy
import threading
import time
import numpy as np
import Queue
from detect_torch import ToyCar
from camera.camera_model import CameraModel
from camera.camera_capture import CameraCap
from create_msgs.msg import laser2map
from std_msgs.msg import Header
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Twist

import actionlib
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from scipy.spatial.transform import Rotation as R

def main():
    rospy.init_node("detect_toycar")

    detect_param = rospy.get_param("~detect")
    camera_param = rospy.get_param("~camera")
    camera_param_root = rospy.get_param("~camera_param_root")

    detect_interval = rospy.get_param("~detect_interval")
    pub = rospy.Publisher('toycar_info', toycar_frame, queue_size=1)

    cam_param_root = os.path.join(camera_param_root,camera_param['far_camera']['path'])

    far_cap = CameraCap('far_camera',camera_param,camera_param_root)
    near_cap = CameraCap('near_camera',camera_param,camera_param_root)

    detect = ToyCar(**detect_param)

    cur_index = 0
    while not rospy.is_shutdown():
        ret, img = cap.read()
        if not ret:
            continue
        # 间隔检测
        cur_index +=1
        if cur_index%detect_interval!=0:
            cv2.imshow('debug', img)
            cv2.waitKey(1)
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
        pub_info = [toycar(box=b,confidence=c,position=[*p[0],*p[1]]) for b,c,p in zip(box,conf,pos)]
        pub.publish(toycar_frame(header=header,toycar_info=pub_info))


class push_toycar():
    def __init__(self,detect, far_cap, near_cap,patrol_route):
        self.detect = detect
        self.far_cap = far_cap
        self.near_cap = near_cap
        self.patrol_route = patrol_route
        #
        self.RT = Queue.Queue(10)

        self.move_base = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(5))

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        threads = [threading.Thread(target=self.listen_RT)]
        threads.append(threading.Thread(target=self.run))
        for t in threads:
            t.start()
        # for t in threads:
        #     t.join()

    def run(self):
        while not rospy.is_shutdown():
            # 找到小车
            map_pos = self.find_toycar()
            # 移动到距离小车15cm
            self.move(map_pos)
            # 机器人小车对接
            self.docking_toycar()
            # 将小车推送到指定地点
            self.push2target()

    def callback(self,data,q):
        q.put(data)
        q.get() if q.qsize()>1 else time.sleep(0.02)

    def listen_RT(self,):
        rospy.Subscriber("laser2map", laser2map, self.callback,self.RT)
        rospy.spin()

    def move(self,pos,max_time = 60):
        # todo 计算合适的位置
        #      下面摄像头检测到小车

        self.move_base.cancel_goal()
        goal = MoveBaseGoal()
        header = Header(999, rospy.Time.now(), 'map')
        goal.target_pose.header = header
        goal.target_pose.pose = Pose(Point(pos[0], pos[1], 0), Quaternion(0, 0, 0, 1))
        self.move_base.send_goal(goal)
        t = 0
        while not rospy.is_shutdown() and t<max_time:

            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                return
            t+=1
            time.sleep(1)

    # todo 需要多帧确认
    # todo 确认后，停下
    def find_toycar(self):
        # 找到小车
        # stage 1 旋转找车
        cur_turn = 0.01
        max_turn = 0.1

        RT = self.RT.get()
        init_theta = R.from_matrix(np.array(RT.R).reshape(3,3)).as_euler('zxy',degrees=True)

        while not rospy.is_shutdown():
            twist = Twist()
            twist.linear = [0,0,0]
            if cur_turn>=max_turn:
                twist.angular = [0, 0, cur_turn]
            else:
                twist.angular = [0, 0, cur_turn + 0.01]

            # 检测
            img = self.far_cap.read()
            box,conf = self.detect(img)
            #
            RT = self.RT.get()
            if len(box)>0:
                points = [[[(b[0] + b[2]) / 2, b[3]]] for b in box]
                pos = self.far_cap.get_position(points)
                R_ = np.array(RT.R).reshape(3,3)
                T_ = np.array(RT.T).reshape(3,1)
                map_pos = [R_.dot(p)+T_ for p in np.array(pos).reshape(-1,3,1)]
                return map_pos
            cur_theta = R.from_matrix(np.array(RT.R).reshape(3,3)).as_euler('zxy',degrees=True)
            if cur_theta - init_theta>350:
                break
            self.cmd_vel_pub.publish(twist)
        # 停下
        while cur_turn>0:
            twist = Twist()
            twist.linear = [0, 0, 0]
            cur_turn = min(cur_turn - 0.01, 0)
            twist.angular = [0, 0, cur_turn]
            self.cmd_vel_pub.publish(twist)
        # stage 2 按照规定的路线找车
        for idx,tp in enumerate(self.patrol_route):
            self.move_base.cancel_goal()
            goal = MoveBaseGoal()
            header = Header(idx, rospy.Time.now(), 'map')
            goal.target_pose.header = header
            goal.target_pose.pose = Pose(Point(tp[0], tp[1], 0), Quaternion(0, 0, 0, 1))
            self.move_base.send_goal(goal)

            while not rospy.is_shutdown():

                state = self.move_base.get_state()
                if state == GoalStatus.SUCCEEDED:
                    break
                # elif state == GoalStatus.ABORTED:
                #     self.move_base.send_goal(goal)

                # 检测
                img = self.far_cap.read()
                box, conf = self.detect(img)
                #
                RT = self.RT.get()
                if len(box) > 0:
                    points = [[[(b[0] + b[2]) / 2, b[3]]] for b in box]
                    pos = self.far_cap.get_position(points)
                    R_ = np.array(RT.R).reshape(3, 3)
                    T_ = np.array(RT.T).reshape(3, 1)
                    map_pos = [R_.dot(p) + T_ for p in np.array(pos).reshape(-1, 3, 1)]
                    return map_pos

    def docking_toycar(self):
        pass

    def push2target(self):
        pass



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
    had_pub = False
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
        # 只发布一个目标
        if not had_pub:

            had_pub = True

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
