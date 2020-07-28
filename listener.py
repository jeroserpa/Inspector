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

            grayscale = np.array(depth_image/np.max(depth_array)*255,dtype = np.uint8)
            
            #"better" histogram equalization
            # clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
            # cl1 = clahe.apply(grayscale)

            grayscale_h = cv2.equalizeHist(grayscale.astype(np.uint8))
            blur = cv.GaussianBlur(grayscale_h,(29,29),0)
            #denoised =cv.fastNlMeansDenoising(grayscale_h,10,7,21)
            #gradient(blur)
            #print(depth_array.shape, np.max(depth_array))
            cv.imshow('depth',depth_image)
            
            # cv2.namedWindow("gray_h")
            # cv.imshow('gray_h',blur)
            
            # cv2.namedWindow("gray")
            # cv.imshow('gray',grayscale)
           
            cv2.waitKey(1)
            print(np.max(depth_array))
            #print ('center depth:', depth_array[center_idx[0], center_idx[1]])


        except CvBridgeError, e:
            print e
        #Convert the depth image to a Numpy array
        
    n+=1
    


def gradient(image):
    h,w = image.shape
    img_sx = cv2.Sobel(image, cv2.CV_32F, 1, 0, ksize=3)
    img_sy = cv2.Sobel(image, cv2.CV_32F, 0, 1, ksize=3)
    image_bgr = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR) 
    image_bgr_mean = image_bgr.copy()
    mag, angle = cv2.cartToPolar(img_sx, img_sy, angleInDegrees=False)
    a_density = 20
    dy=(int)(h/a_density)
    dx=(int)(w/a_density)
    for y in range((int)(dy/2),h,dy):
        for x in range((int)(dx/2),w,dx):
            start = (x,y)
            d_x = (int)(mag[y][x]*np.cos(angle[y][x]))
            d_y = (int)(mag[y][x]*np.sin(angle[y][x]))
        #  dx = (int)(20*np.cos(angle[y][x]))
        #   dy = (int)(20*np.sin(angle[y][x])) 
            end = ( x + d_x , y + d_y)


            image_bgr=cv2.arrowedLine(image_bgr,start,end,(0, 0, 255),thickness=2 ,tipLength = 0.2)


    density = 10 #1; 2; 3; 4; 5; 6; 8; 10; 12; 15; 20; 24; 30; 40; 60 COMMON FACTORSS
    i=0
    dy=(int)(h/density)
    dx=(int)(w/density)
    x=0
    y=0
    blocks_ang = np.zeros([dy,dx,density*density])
    blocks_mag = np.zeros([dy,dx,density*density])

    #store each segment in n array
    for y in range(0,h,dy):
        for x in range(0,w,dx):

            blocks_ang[:,:,i]=angle[y:y+dy,x:x+dx]
            blocks_mag[:,:,i]=mag[y:y+dy,x:x+dx]
            i+=1
    #draw grid
    for y in range(0,h,dy):
        cv2.line(image_bgr_mean, (0, y), (w, y), (255, 0, 0), 1, 1)
    for x in range(0,w,dx):
        cv2.line(image_bgr_mean, (x, 0), (x, h), (255, 0, 0), 1, 1)


    means = np.zeros([blocks_ang.shape[2],2])#gx,gy
    #we calculate the average angle fo the segment

    for i in range(0,means.shape[0]):
        av_cos = np.mean( np.multiply(blocks_mag[...,i],np.cos(blocks_ang[...,i]))) 
        av_sin = np.mean( np.multiply(blocks_mag[...,i],np.sin(blocks_ang[...,i])))  
        means[i,0] = av_cos
        means[i,1] = av_sin
        # if means_N[i,0] != 0 :     
        #   means_N[i,0] = means_N[i,1] / np.max(means_N[i,0])
        # if means_N[i,1] != 0 :     
        #   means_N[i,1] = means_N[i,1] / np.max(means_N[i,1])    

        i=0
        dy=(int)(h/density)
        dx=(int)(w/density)
    #we draw de gradients
    for y in range((int)(dy/2),h,dy):
        for x in range((int)(dx/2),w,dx):
            start = (x ,y)
            d_x =(int)(means[i][0]*2) #(int)(10*np.cos( means[i]))
            d_y =(int)(means[i][1]*2) #(int)(10*np.sin( means[i]))
            end = ( x + d_x , y + d_y)
        
    
            if (np.sqrt((means[i][0])**2 +  (means[i][1])**2)>2) :
                image_bgr_mean=cv2.arrowedLine(image_bgr_mean,start,end,(0, 0, 255),thickness=2,tipLength = 0.2 )
            else:
                cv2.circle(image_bgr_mean, (x,y), 10,  (255,0,0),thickness=2)
            #image_bgr_mean[y][x] = (0,0,255)
            i+=1
        
        
        cv.imshow('image_bgr',image_bgr_mean)
        cv2.waitKey(1)




def listener():
    n = 0
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('/camera/depth/image_rect_raw/compressed', Image, callback,queue_size=1)
    #rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, callback,queue_size=1)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()





if __name__ == '__main__':
    listener()
