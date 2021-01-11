#!/usr/bin/python

import time
import tf
import rospy
import multiprocessing as mp
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from nav_msgs.msg import Odometry


class fake_localiztion():
    def __init__(self):
        rospy.init_node('fake_localiztion', anonymous=True)
        # self.pub = rospy.Publisher('amcl_pose', PoseWithCovarianceStamped, queue_size=10)
        self.base_odom_br = tf.TransformBroadcaster()
        print('finish init')
        rospy.Subscriber('/odom', Odometry, self.pose_call)
        rospy.spin()
    def pose_call(self,data):
        pose_data = data.pose.pose
        pose = [(pose_data.position.x, pose_data.position.y, pose_data.position.z),
                     (pose_data.orientation.x, pose_data.orientation.y,
                      pose_data.orientation.z, pose_data.orientation.w)]
        self.base_odom_br.sendTransform(pose[0],
                         pose[1],
                         rospy.Time.now(),
                         "base_footprint",
                         "/odom"
                         )

if __name__ == '__main__':
    fake_localiztion()
