#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2 as cv, cv2
import numpy as np
from matplotlib.pyplot import imshow

def gradient(image):
    density = 16 #1; 2; 3; 4; 5; 6; 8; 10; 12; 15; 20; 24; 30; 40; 60 COMMON FACTORSS
    h,w = image.shape
    (img_sx,img_sy) =  N_sobel(image)
    
    image_bgr = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR) 
   
    
    
    avgs = grad_avg(img_sx,img_sy,density)
    
    #image_bgr = draw_grid(image_bgr,density)

    #image_bgr = avg_draw(image_bgr,avgs,100)
   
    image_bgr_dir = dir_draw(image_bgr,avgs)

    return image_bgr_dir
    
def N_sobel(d_image):

  img_sx = cv2.Sobel(d_image, cv2.CV_32F, 1, 0, ksize=3)
  img_sy = cv2.Sobel(d_image, cv2.CV_32F, 0, 1, ksize=3)

  img_sx = img_sx/np.max(img_sx)
  img_sy = img_sy/np.max(img_sy)
  
  return(img_sx,img_sy)


def grad_avg(img_sx,img_sy,density = 10):
    h,w = img_sx.shape
    
    dy=(int)(h/density)
    dx=(int)(w/density)
    i=0
    x=0
    y=0

    blocks_sx = np.zeros([dy,dx,density*density])
    blocks_sy = np.zeros([dy,dx,density*density])

    #store each segment in an array
    for y in range(0,h,dy):
        for x in range(0,w,dx):

            # blocks_ang[:,:,i]=angle[y:y+dy,x:x+dx]
            # blocks_mag[:,:,i]=mag[y:y+dy,x:x+dx]
            blocks_sx[:,:,i]=img_sx[y:y+dy,x:x+dx]
            blocks_sy[:,:,i]=img_sy[y:y+dy,x:x+dx]
            i+=1

    #calculate average
    avgs = np.zeros([blocks_sx.shape[2],2])  #average x, average y  
    for i in range(0,avgs.shape[0]):
        av_x = np.mean(blocks_sx[...,i]) 
        av_y = np.mean(blocks_sy[...,i])  
        avgs[i,0] = av_x
        avgs[i,1] = av_y
    return(avgs)

def draw_grid(image_bgr,density=10):
  h,w,c =image_bgr.shape
  dy=(int)(h/density)
  dx=(int)(w/density)
  for y in range(0,h,dy):
    cv2.line(image_bgr, (0, y), (w, y), (255, 0, 0), 1, 1)
  for x in range(0,w,dx):
    cv2.line(image_bgr, (x, 0), (x, h), (255, 0, 0), 1, 1)
  return(image_bgr)

def avg_draw(image_bgr,avgs,factor=100):
  h,w,c=image_bgr.shape
  i=0
  dy=(int)(h/np.sqrt(avgs.shape[0]))
  dx=(int)(w/np.sqrt(avgs.shape[0]))

  
  for y in range((int)(dy/2),h,dy):
    for x in range((int)(dx/2),w,dx):
      start = (x ,y)
      d_x =(int)(avgs[i][0]*factor) #(int)(10*np.cos( means[i]))
      d_y =(int)(avgs[i][1]*factor) #(int)(10*np.sin( means[i]))
      end = ( x + d_x , y + d_y)
    
     
      if (np.sqrt((avgs[i][0])**2 +  (avgs[i][1])**2)>0.001) :
        image_bgr_mean=cv2.arrowedLine(image_bgr,start,end,(0, 0, 255),thickness=2,tipLength = 0.2 )
      else:
        cv2.circle(image_bgr, (x,y), 10,  (255,0,0),thickness=2)
        
      i+=1
  return(image_bgr)

def dir_draw(image_bgr,avgs,factor=5000):
    h,w,c=image_bgr.shape
    av_X = np.mean(avgs[:,0])*factor  
    av_Y= np.mean(avgs[:,1])*factor
    start = ((int)(w/2),(int)(h/2))
    end = ( (int)(w/2) + (int)(av_X)  , (int)(h/2) + (int)(av_Y)*10)
    if (np.sqrt(av_X**2+av_Y**2)>0.001*factor):
        image_bgr_dir = cv2.arrowedLine(image_bgr,start,end,(255, 0, 0),thickness=2,tipLength = 0.2 )
    else:
        image_bgr_dir = cv2.circle(image_bgr, start, 30,  (255,0,0),thickness=2)
    return(image_bgr_dir)


n = 0
def  callback(ros_image):
    global n
    if n%2 == 0 :
        bridge = CvBridge()
        # Use cv_bridge() to convert the ROS image to OpenCV format
        try:
        #Convert the depth image using the default passthrough encoding
            #bridge returns old style of cv image (ipl) thats why we convert it to array
            depth_original = bridge.imgmsg_to_cv2(ros_image, desired_encoding="passthrough") #rgb8", "bgr8", "rgba8", "bgra8", "mono8" or "mono16"
            depth_array = np.array(depth_original, dtype=np.float32)
            depth_image = depth_array/np.max(depth_array)
            center_idx = np.array(depth_array.shape) / 2

            depth_uint8 = np.array(depth_image*255,dtype = np.uint8)
            
            #"better" histogram equalization
            # clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
            # cl1 = clahe.apply(grayscale)

            depth_eq = cv2.equalizeHist(depth_uint8)
            blur_eq = cv.GaussianBlur(depth_eq,(21,21),0)
            blur = cv.GaussianBlur(depth_image,(9,9),0)
                        
            #blur=np.abs(blur-1)
            dir = gradient(blur)
            dir_eq = gradient(blur_eq)
            #print(depth_array.shape, np.max(depth_array))
            cv.imshow('depth',dir)
            cv.imshow('depth_eq',dir_eq)
            
            cv2.waitKey(1)
            
            #print ('center depth:', depth_array[center_idx[0], center_idx[1]])

        except CvBridgeError as e:
            print (e)
        #Convert the depth image to a Numpy array
    n+=1

def listener():
    n = 0
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, callback,queue_size=1)
    #rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, callback,queue_size=1)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()



if __name__ == '__main__':
    listener()
