#!/usr/bin/env python

import rospy

from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

path_ekf = Path()
path_t265 = Path()


def path_publisher():

    global k
    k = 0
    new_ekf_pose = False
    global new_t265_pose 
    new_t265_pose = True
    def odom_ekf_cb(data):
        global path_ekf
        global k
        if k % 3 == 0:
            path_ekf.header = data.header
            pose = PoseStamped()
            pose.header = data.header
            pose.pose = data.pose.pose
            path_ekf.poses.append(pose)
            path_ekf_pub.publish(path_ekf)
            new_ekf = True
        k+=1

 
    global i 
    i = 0
    def odom_t265_cb(data):
        global i
        global path_t265
        global new_t265_pose
        if i % 20 == 0:
            
            path_t265.header = data.header
            pose = PoseStamped()
            pose.header = data.header
            pose.pose = data.pose.pose
            path_t265.poses.append(pose)
            path_t265_pub.publish(path_t265)
            new_t265_pose = True
            #print(new_t265_pose)

        i+=1
            
           
   
        
    

    rospy.init_node('path_node')
    #for EKF
    odom_ekf_sub = rospy.Subscriber('odometry/filtered', Odometry, odom_ekf_cb)
    path_ekf_pub = rospy.Publisher('path/ekf', Path, queue_size=10)
    #t265
    odom_t265_sub = rospy.Subscriber('t265/odom/sample', Odometry, odom_t265_cb)
    path_t265_pub = rospy.Publisher('path/t265', Path, queue_size = 5)
    rospy.spin()
 


if __name__ == '__main__':
   path_publisher()