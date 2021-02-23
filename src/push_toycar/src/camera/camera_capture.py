import os
import cv2
from camera.camera_model import CameraModel

class CameraCap():
    def __init__(self,camera_name,camera_param,camera_param_root):
        cam_param_root = os.path.join(camera_param_root, camera_param[camera_name]['path'])
        self.cam_model = CameraModel(cam_param_root)

        cap = cv2.VideoCapture(camera_param[camera_name]['dev'])
        cap.set(5, camera_param['camera_fps'])
        cap.set(3, int(camera_param['image_shape'][0]))
        cap.set(4, int(camera_param['image_shape'][1]))
        print('img height :', cap.get(3))
        print('img width:', cap.get(4))
        print('img fps:', cap.get(5))
        self.cap = cap

    def read(self):
        return self.cap.read()

    def get_position(self,points):
        if len(points) == 0:
            pos = []
        else:
            pos = self.cam_model.run(points)
        return pos
