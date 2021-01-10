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


def listen_rviz_init_pose(q):
    rospy.init_node('rviz_init_pose_listen', anonymous=True)
    rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, listen_rviz_init_pose_callback, callback_args=q)
    rospy.spin()

def listen_rviz_init_pose_callback(data,q):
    q.put(data)
    q.get() if q.qsize() > 1 else time.sleep(0.001)

def main():
    fake_localiztion()
    '''
    mp.set_start_method(method='spawn')  # init fork spawn forkserver
    init_pose_q = mp.Queue(maxsize=4)

    processes = [mp.Process(target=listen_rviz_init_pose, args=(init_pose_q, ))]
    processes.append(mp.Process(target=fake_localiztion, args=(init_pose_q, )))

    for process in processes:  # debug
        process.daemon = True  # setattr(process, 'deamon', True)
        process.start()  # debug
    for process in processes:
        process.join()
    '''

def test():
    listen_rviz_init_pose([])

if __name__ == '__main__':
    main()
    # test()