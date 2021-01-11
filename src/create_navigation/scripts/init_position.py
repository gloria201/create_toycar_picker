#!/usr/bin/python

import time
import tf
from tf.transformations import quaternion_inverse, quaternion_multiply, quaternion_matrix,quaternion_from_matrix
import numpy as np
import rospy
import threading
import queue
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from nav_msgs.msg import Odometry

def compute_pose(pose1,pose2):
    H1 = quaternion_matrix(pose1[1])
    H2 = quaternion_matrix(pose2[1])
    H1[:3,3] = pose1[0]
    H2[:3,3] = pose2[0]
    H = H1.dot(np.linalg.inv(H2))
    pose = [H[:3,3].tolist(),quaternion_from_matrix(H).tolist()]
    return pose


def odom_map_pos(init_pose_q,odom_topic_q):
    init_pose = [[0.000, 0.000, 0.000], [0.000, 0.000, 0, 1]]
    # self.pub = rospy.Publisher('amcl_pose', PoseWithCovarianceStamped, queue_size=10)
    base_odom_br = tf.TransformBroadcaster()


    while 1:
        if init_pose_q.qsize()>0:
            init_pose_data = init_pose_q.get()
            init_pose_data = init_pose_data.pose.pose
            init_pose = [(init_pose_data.position.x,init_pose_data.position.y,init_pose_data.position.z),
                         (init_pose_data.orientation.x,init_pose_data.orientation.y,
                          init_pose_data.orientation.z,init_pose_data.orientation.w)]

            odom_pose_data = odom_topic_q.get()
            odom_pose_data = odom_pose_data.pose.pose
            odom_pose = [(odom_pose_data.position.x, odom_pose_data.position.y, odom_pose_data.position.z),
                    (odom_pose_data.orientation.x, odom_pose_data.orientation.y,
                     odom_pose_data.orientation.z, odom_pose_data.orientation.w)]

            init_pose = compute_pose(init_pose,odom_pose)
            print('init odom pose', init_pose)
        time_now = rospy.Time.now()
        base_odom_br.sendTransform(init_pose[0],
                         init_pose[1],
                         time_now,
                         "odom",
                         "map")
        time.sleep(0.03)


def listen_rviz_init_pose(q):
    rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, listen_rviz_init_pose_callback, callback_args=q)
    rospy.spin()

def listen_rviz_init_pose_callback(data,q):
    q.put(data)
    q.get() if q.qsize() > 1 else time.sleep(0.001)

def listen_odom_topic(q):
    rospy.Subscriber('/odom', Odometry,listen_rviz_init_pose_callback,callback_args=q)
    rospy.spin()

def listen_odom_topic_callback(data,q):
    q.put(data)
    q.get() if q.qsize() > 1 else time.sleep(0.001)

def main():
    # mp.set_start_method(method='spawn')  # init fork spawn forkserver
    rospy.init_node('pub_odom_map', anonymous=True)
    init_pose_q = queue.Queue(maxsize=4)
    odom_pose_q = queue.Queue(maxsize=4)

    processes = [threading.Thread(target=listen_rviz_init_pose, args=(init_pose_q, ))]
    processes.append(threading.Thread(target=odom_map_pos, args=(init_pose_q,odom_pose_q, )))
    processes.append(threading.Thread(target=listen_odom_topic, args=(odom_pose_q, )))

    for process in processes:  # debug
        process.daemon = True  # setattr(process, 'deamon', True)
        process.start()  # debug
    for process in processes:
        process.join()
def test():
    listen_rviz_init_pose([])

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
