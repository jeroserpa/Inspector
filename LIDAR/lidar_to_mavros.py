#!/usr/bin/env python
import roslib; roslib.load_manifest('rviz')
import rospy
from sensor_msgs.msg import Range



def talker():
    pub0 = rospy.Publisher('/mavros/distance_sensor/rangefinder_0_sub', Range, queue_size = 10)
    pub45 = rospy.Publisher('/mavros/distance_sensor/rangefinder_45_sub', Range, queue_size = 10)
    rospy.init_node('talker')
    # ranges = [float('NaN'), 1.0, -float('Inf'), 3.0, float('Inf')]
    ranges = [float(1)]
    min_range = 0.04
    max_range = 4
    while not rospy.is_shutdown():    
        r = Range()
        r.header.stamp = rospy.Time.now()
        r.header.frame_id = "/base_link"
        r.radiation_type = 1 # 0 ultrasound - 1 infrared
        r.field_of_view = 0.349066 #radian
        r.min_range = min_range #[m]
        r.max_range = max_range #[m]
        r.range = rg
        pub.publish(r)
        rospy.sleep(1.0)
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass