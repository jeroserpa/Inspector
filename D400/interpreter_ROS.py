import numpy as np                        # fundamental package for scientific computing
import matplotlib.pyplot as plt           # 2D plotting library producing publication quality figures
import pyrealsense2 as rs                 # Intel RealSense cross-platform open-source API
import cv2 as cv,cv2
import time
import openspace as op
import depthgradient_ROS as gd
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

print("Environment Ready")

bridge = CvBridge()


def Publisher(avg):
    
    pub = rospy.Publisher('avg', String, queue_size=10)
    #rospy.init_node('Interpreter', anonymous=True)
    rate = rospy.Rate(5) # 5hz
    if not rospy.is_shutdown():
          message = str(avg["Tx"])
          rospy.loginfo(message)
          pub.publish(message)
          
          
def image_callback(msg):
  
  try:
      # Convert your ROS Image message to OpenCV2
      cv2_img = bridge.imgmsg_to_cv2(msg)
      depth_array = np.array(cv2_img, dtype=np.float32)
      depth_array = depth_array/np.max(depth_array)
  except CvBridgeError, e:
      print(e)
  else:
        avg = gd.process(depth_array)
        try:
            Publisher(avg)
        except rospy.ROSInterruptException:
            pass
    

def main():

   
    
    start_time = time.time() #to check preformance 
    rospy.init_node("Gradient")
    rate = rospy.Rate(5)
    rospy.Subscriber("/d435/depth/image_rect_raw", Image, image_callback)


    
    rospy.spin()
     
        



if __name__ == '__main__':
    main()