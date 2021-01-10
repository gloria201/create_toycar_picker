import time
import tf
import rospy
import multiprocessing as mp
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from nav_msgs.msg import Odometry



def odom_map_pos(init_pose_q):
    rospy.init_node('odom_map', anonymous=True)
    init_pose = [[0.000, 0.300, 0.000], [0.000, 0.000, 0.223, 0.975]]
    # self.pub = rospy.Publisher('amcl_pose', PoseWithCovarianceStamped, queue_size=10)
    base_odom_br = tf.TransformBroadcaster()


    while 1:
        if init_pose_q.qsize()>0:
            init_pose_data = init_pose_q.get()
            init_pose_data = init_pose_data.pose.pose
            init_pose = [(init_pose_data.position.x,init_pose_data.position.y,init_pose_data.position.z),
                         (init_pose_data.orientation.x,init_pose_data.orientation.y,
                          init_pose_data.orientation.z,init_pose_data.orientation.w)]
            print('init odom pose', init_pose)
        time_now = rospy.Time.now()
        base_odom_br.sendTransform(init_pose[0],
                         init_pose[1],
                         time_now,
                         "odom",
                         "map")
        time.sleep(0.03)


def listen_rviz_init_pose(q):
    rospy.init_node('rviz_init_pose_listen', anonymous=True)
    rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, listen_rviz_init_pose_callback, callback_args=q)
    rospy.spin()

def listen_rviz_init_pose_callback(data,q):
    q.put(data)
    q.get() if q.qsize() > 1 else time.sleep(0.001)

def main():
    # mp.set_start_method(method='spawn')  # init fork spawn forkserver
    init_pose_q = mp.Queue(maxsize=4)

    processes = [mp.Process(target=listen_rviz_init_pose, args=(init_pose_q, ))]
    processes.append(mp.Process(target=odom_map_pos, args=(init_pose_q, )))

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