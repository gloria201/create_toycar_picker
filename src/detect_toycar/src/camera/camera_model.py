#!/usr/bin/python3
# -*- coding: utf-8 -*-
'''
TODO 近处没有激光标定的时候
'''
import os
import pickle
import numpy as np

class CameraModel():
    def __init__(self,camera_param_root = ''):
        inter_param = pickle.load(open(os.path.join(camera_param_root,'inter_param.pkl'),'rb'))
        self.K = inter_param['K']
        self.D = inter_param['D']
        exter_param = pickle.load(open(os.path.join(camera_param_root, 'exter_param.pkl'), 'rb'))
        self.rvec = exter_param['rvec']
        self.tvec = exter_param['tvec']

        laser_param_path = os.path.join(camera_param_root, 'Laser2Camera.pkl')
        if os.path.exists(laser_param_path):
            laser_param = pickle.load(open(laser_param_path,'rb'))
            R_TCL = laser_param['R']
            T_TCL = laser_param['T']
            self.R_TLC = np.linalg.inv(R_TCL)
            self.T_TLC = -np.linalg.inv(R_TCL).dot(T_TCL.reshape(3,1))

    def run(self,boxes):
        positions = []
        # for box in boxes:
        box = boxes[0]
        point = self.pixel2cam_world([[box[0],box[3]],[box[2],box[3]]])
        print('pooint',point)
        positions=self.cam_world2laser(point)
        return positions

    def cam_world2laser(self,points):
        laser_pos = []
        for p in points:
            laser_p = self.R_TLC.dot(np.array(p).reshape(3,1)) + self.T_TLC.reshape(3,1)
            laser_pos.append(laser_p.flatten().tolist())
        return laser_pos

    def pixel2cam_world(self, points):
        points = np.array(points)
        points = np.hstack([points, np.ones((points.shape[0], 1))])
        world_points = []
        Rvec_inv = np.linalg.inv(self.rvec)
        Z_w = 0
        for point in points:
            M1 = Rvec_inv.dot(point).flatten()
            M2 = Rvec_inv.dot(self.tvec.reshape(3, 1)).flatten()
            Zc = (Z_w + M2[2]) / M1[2]
            x = (Zc * M1[0] - M2[0])
            y = (Zc * M1[1] - M2[1])
            cam_p = self.rvec.dot(np.array([[x], [y], [0]])) + self.tvec
            cam_p = np.squeeze(cam_p).tolist()
            world_points.append(cam_p)

        return np.array(world_points)
