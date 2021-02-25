#!/home/gloria/anaconda3/envs/py3_8_env/bin/python3
# -*- coding: utf-8 -*-

import os
import cv2
import rospy
import threading
import time
import numpy as np
import queue
from detect_torch import ToyCar
from camera.camera_model import CameraModel
from camera.camera_capture import CameraCap
from create_msgs.msg import laser2map
from std_msgs.msg import Header
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Twist, Vector3

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
    docking_toycar_params = rospy.get_param("~docking_toycar")
    find_toycar_params = rospy.get_param("~find_toycar")
    detect_interval = rospy.get_param("~detect_interval")
    final_goal = rospy.get_param("~final_goal")

    far_cap = CameraCap('far_camera',camera_param,camera_param_root)
    near_cap = CameraCap('near_camera',camera_param,camera_param_root)

    detect = ToyCar(**detect_param)

    push_toycar(detect,far_cap,near_cap,find_toycar_params,docking_toycar_params,final_goal)

class push_toycar():
    def __init__(self,detect, far_cap, near_cap,find_toycar_params,docking_toycar_params,final_goal):
        self.detect = detect
        self.far_cap = far_cap
        self.near_cap = near_cap
        self.find_toycar_params = find_toycar_params
        self.final_goal = final_goal
        #
        self.RT = queue.Queue(10)

        self.move_base = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(5))

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.docking_toycar_params = docking_toycar_params

        threads = [threading.Thread(target=self.listen_RT)]
        threads.append(threading.Thread(target=self.run))
        try:
            for t in threads:
                t.start()
        finally:
            twist = Twist()
            twist.linear = Vector3(0, 0, 0)
            twist.angular = Vector3(0, 0, 0)
            self.cmd_vel_pub.publish(twist)
        # for t in threads:
        #     t.join()

    def run(self):
        self.window_name = 'test_windows'
        cv2.namedWindow(self.window_name, 0)
        while not rospy.is_shutdown():
            # 找到小车
            map_pos = self.find_toycar()
            print('Finded toycar and start to move to toycar')
            # 移动到距离小车15cm
            self.move(map_pos)
            print('arrival position and start to dock ')
            # 机器人小车对接
            self.docking_toycar()
            print('docked toycar  and start push toycar to target position')
            # 将小车推送到指定地点
            # self.push2target()
            print('fininsh push ')

    def callback(self,data,q):
        q.put(data)
        q.get() if q.qsize()>1 else time.sleep(0.02)

    def listen_RT(self,):
        rospy.Subscriber("laser2map", laser2map, self.callback,self.RT)
        rospy.spin()

    def move(self,pos,max_time = 60):
        '''
        得到了toycar的位置
           移动到距离小车t_dis距离的位置并且面朝toycar,并且满足近摄像头检测到小车
        '''
        self.move_base.cancel_goal()
        goal = MoveBaseGoal()
        header = Header(999, rospy.Time.now(), 'map')
        goal.target_pose.header = header

        RT = self.RT.get()
        T_= RT.T
        t_dis = 0.35
        dis = ((T_[0]-pos[0])**2+(T_[1]-pos[1])**2)**0.5
        # 移动到的位置一定大于当前与目标的位置
        assert dis>t_dis
        ratio = t_dis/dis
        move_pose_position = [(T_[0]-pos[0])*ratio+pos[0],(T_[1]-pos[1])*ratio+pos[1],0]

        theta = np.arccos((pos[0]-T_[0])/dis)
        if pos[1]-T_[1]<0:
            theta = 360-theta
        r = R.from_euler('zxy',(theta,0,0))
        move_pose_orientation = r.as_quat()


        goal = MoveBaseGoal()
        header = Header(888, rospy.Time.now(), 'map')
        goal.target_pose.header = header
        goal.target_pose.pose = Pose(Point(*move_pose_position), Quaternion(*move_pose_orientation))
        self.move_base.send_goal(goal)
        t = 0
        while not rospy.is_shutdown() and t<max_time:

            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                break
            t+=1
            time.sleep(1)

        # 近距离的相机检测小车确认
        img = self.near_cap.read()
        box, conf = self.detect.run(img)
        if len(box)>0:
            return True
        else:
            rospy.logerr(' Near Camera No Find Toycar ')
            rospy.logerr(' maybe toycar position is wrong ')

    # todo 需要多帧确认
    # todo 确认后，停下, 调整成朝向目标
    def find_toycar(self):
        '''
        按照既定的路线移动，移动过程中寻找小车
            首先旋转一圈，
            再按照既定路线移动，配置中patrol_route
            移动一圈都没有找到，后报错，结束
        '''
        self.target_check = target_check()

        # 找到小车
        # stage 1 旋转找车
        cur_turn = 0.01
        max_turn = 0.1

        RT = self.RT.get()
        init_theta = R.from_matrix(np.array(RT.R).reshape(3,3)).as_euler('zxy',degrees=True)

        while not rospy.is_shutdown():
            twist = Twist()
            twist.linear = Vector3(0,0,0)
            if cur_turn>=max_turn:
                twist.angular = Vector3(0, 0, cur_turn)
            else:
                cur_turn = cur_turn + 0.01
                twist.angular = Vector3(0, 0, cur_turn)

            # 检测
            #print('to get image')
            img = self.far_cap.read()
            box,conf = self.detect.run(img)
            #print(box,conf)
            for b in box:
                cv2.rectangle(img, (int(b[0]), int(b[1])), (int(b[2]), int(b[3])), (0, 254, 0), 1)
            cv2.imshow(self.window_name, img)
            cv2.waitKey(1)
            #
            RT = self.RT.get()
            if len(box)>0:
                points = [[[(b[0] + b[2]) / 2, b[3]]] for b in box]
                pos = self.far_cap.get_position(points)
                R_ = np.array(RT.R).reshape(3,3)
                T_ = np.array(RT.T).reshape(3,1)
                map_pos = [(R_.dot(p)+T_).flatten().tolist() for p in np.array(pos).reshape(-1,3,1)]
                if self.target_check.update(RT.header.stamp.to_sec(),map_pos):
                    return self.target_check.get_target()
            cur_theta = R.from_matrix(np.array(RT.R).reshape(3,3)).as_euler('zxy',degrees=True)


            if cur_theta[0] >=(init_theta[0]-5) and cur_theta[0] - init_theta[0]>320:
                break
            elif cur_theta[0] <(init_theta[0]-5) and (180- init_theta[0])+(cur_theta[0]+180)>320:
                break
            self.cmd_vel_pub.publish(twist)
        # 停下
        while cur_turn>0:
            twist = Twist()
            twist.linear = Vector3(0, 0, 0)
            cur_turn = min(cur_turn - 0.01, 0)
            twist.angular = Vector3(0, 0, cur_turn)

            # 检测
            img = self.far_cap.read()
            box,conf = self.detect.run(img)
            for b in box:
                cv2.rectangle(img, (int(b[0]), int(b[1])), (int(b[2]), int(b[3])), (0, 254, 0), 1)
            cv2.imshow(self.window_name, img)
            cv2.waitKey(1)
            #
            RT = self.RT.get()
            if len(box)>0:
                points = [[[(b[0] + b[2]) / 2, b[3]]] for b in box]
                pos = self.far_cap.get_position(points)
                R_ = np.array(RT.R).reshape(3,3)
                T_ = np.array(RT.T).reshape(3,1)
                map_pos = [(R_.dot(p)+T_).flatten().tolist() for p in np.array(pos).reshape(-1,3,1)]
                if self.target_check.update(RT.header.stamp.to_sec(), map_pos):
                    return self.target_check.get_target()

            self.cmd_vel_pub.publish(twist)

        print('Finish rotate (not found toycar) and start to patrol')
        # stage 2 按照规定的路线找车
        for idx,tpose in enumerate(self.find_toycar_params['patrol_route']):
            self.move_base.cancel_goal()
            goal = MoveBaseGoal()
            header = Header(idx, rospy.Time.now(), 'map')
            goal.target_pose.header = header
            goal.target_pose.pose = Pose(Point(*tpose[:3]), Quaternion(*tpose[3:]))
            self.move_base.send_goal(goal)

            while not rospy.is_shutdown():

                state = self.move_base.get_state()
                if state == GoalStatus.SUCCEEDED:
                    break
                # elif state == GoalStatus.ABORTED:
                #     self.move_base.send_goal(goal)

                # 检测
                img = self.far_cap.read()
                box, conf = self.detect.run(img)

                for b in box:
                    cv2.rectangle(img, (int(b[0]), int(b[1])), (int(b[2]), int(b[3])), (0, 254, 0), 1)
                cv2.imshow(self.window_name, img)
                cv2.waitKey(1)
                #
                RT = self.RT.get()
                if len(box) > 0:
                    points = [[[(b[0] + b[2]) / 2, b[3]]] for b in box]
                    pos = self.far_cap.get_position(points)
                    R_ = np.array(RT.R).reshape(3, 3)
                    T_ = np.array(RT.T).reshape(3, 1)
                    map_pos = [(R_.dot(p) + T_).flatten().tolist() for p in np.array(pos).reshape(-1, 3, 1)]
                    if self.target_check.update(RT.header.stamp.to_sec(), map_pos):
                        return self.target_check.get_target()

        rospy.logerr('finish patrol and  no found toycar')

    def docking_toycar(self):
        '''
        TODO: 改成pid
        近距离的相机已经能看到目标了,保证只有一个目标
            控制机器人和小车链接
        '''

        cur_turn = 0.05
        max_turn = 0.1
        cur_x = 0.05
        max_x = 0.1

        min_y = min(self.docking_toycar_params['left_port'][1],self.docking_toycar_params['left_port'][1])
        left_x = self.docking_toycar_params['left_port'][0]
        right_x = self.docking_toycar_params['right_port'][0]
        enter_y = self.docking_toycar_params['enter_port']

        while not rospy.is_shutdown():
            twist = Twist()
            twist.angular = Vector3(0, 0, 0)
            twist.linear = Vector3(0, 0, 0)

            img = self.near_cap.read()
            box, conf = self.detect.run(img)

            cv2.line(img, (left_x, 0), (left_x, img.shape[0]), (0,0,255), 2)
            cv2.line(img, (right_x, 0), (right_x, img.shape[0]), (0,0,255), 2)
            cv2.line(img, (0, enter_y), (img.shape[1],enter_y), (0,0,255), 2)
            for b in box:
                cv2.rectangle(img, (int(b[0]), int(b[1])), (int(b[2]), int(b[3])), (0, 254, 0), 1)
            cv2.imshow(self.window_name, img)
            cv2.waitKey(1)

            if len(box)>1:
                rospy.logerr('docking_toycar: more than one toycar')
                assert NotImplementedError
            elif len(box)==0:
                rospy.logerr('docking_toycar: No Find toycar')
                assert NotImplementedError

            if box[3]<min_y:
                if box[2]>right_x:
                    # 左转
                    twist.angular = [0, 0, -cur_turn]
                elif box[0]<right_x:
                    #　右转
                    twist.angular = [0, 0, cur_turn]
                else:
                    # 前进
                    twist.linear = [cur_x, 0, 0]
            else:

                if box[2]<right_x and box[0]>left_x:
                    if box[1]>enter_y:
                        # 表示已经完成
                        self.cmd_vel_pub.publish(twist)
                        return True
                    else:
                        # 前进
                        twist.linear = [cur_x, 0, 0]
                # 出现这种情况，应该哪里没有处理好
                rospy.logwarn('docking_toycar: No Find toycar')
                # 后退
                twist.linear = [-cur_x, 0, 0]

            self.cmd_vel_pub.publish(twist)

    def push2target(self):
        pass

class target_check():
    def __init__(self,max_time= 1, max_distance = 0.1, min_target_times=1):
        self.max_time = max_time # 最大间隔时间
        self.min_target_times = min_target_times # 最小检测次数
        self.max_distance = max_distance # 最大间隔距离
        self.target_info = []# [[time,position]]
        self.target_position = None

    def get_target(self):
        return self.target_position

    def update(self, time, position):
        # 检查时间
        new_target_info = []
        for t in self.target_info:
            if abs(t-t[0])<self.max_time:
                new_target_info.append(t)

        # 检查距离
        self.target_info =[]
        for pos in position:
            for _,p in new_target_info:
                if self.check_distance(pos,p[-1]):
                    p.append(pos)
                    self.target_info.append([time,p])
                    continue
            #
            self.target_info.append([time,[pos]])
        return self.check()

    def check_distance(self,target,pred):
        dis = np.linalg.norm(np.array(target)-np.array(pred))
        if dis<=self.max_distance:
            return True
        else:
            return False

    def check(self):
        for _,p in self.target_info:
            if len(p)>=self.min_target_times:
                self.target_position = p[-1]
                return True
        return False

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
    main()
    # test()