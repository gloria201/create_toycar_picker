#!/usr/bin/python2
# -*- coding: utf-8 -*-
#  将laser和map的转换发布出来
import tf
import rospy
from create_msgs.msg import laser2map
from std_msgs.msg import Header
from tf.transformations import quaternion_matrix

def main():
    rospy.init_node("laser2map")
    pub = rospy.Publisher('laser2map', laser2map, queue_size=1)
    listener = tf.TransformListener()
    cur_index = 0
    r = rospy.Rate(20)

    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/map', '/base_scan', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        header = Header(cur_index, rospy.Time.now(), 'laser2map')
        R = quaternion_matrix(rot)
        pub.publish(laser2map(header=header,R=R.flatten().tolist(),T=trans))
        cur_index += 1
        r.sleep()


if __name__ == '__main__':
    main()