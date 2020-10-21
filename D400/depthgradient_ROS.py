#gradient calculation module

import numpy as np                        # fundamental package for scientific computing
import matplotlib.pyplot as plt           # 2D plotting library producing publication quality figures
import pyrealsense2 as rs                 # Intel RealSense cross-platform open-source API
import cv2 as cv,cv2
from cv_bridge import CvBridge, CvBridgeError
import time

import rospy
from sensor_msgs.msg import Image

#initialize global buffers for moving average 
global grad_bufferXL
global grad_bufferXR
buffer_size=10
grad_bufferXL =  np.arange(0,buffer_size,1)
grad_bufferXR =  np.arange(0,buffer_size,1)

# Instantiate CvBridge
bridge = CvBridge()

def image_callback(msg):
  
  try:
      # Convert your ROS Image message to OpenCV2
      cv2_img = bridge.imgmsg_to_cv2(msg)
      depth_array = np.array(cv2_img, dtype=np.float32)
      depth_array = depth_array/np.max(depth_array)
  except CvBridgeError, e:
      print(e)
  else:
      process(depth_array)
def N_sobel(d_image): #calculates sobel in x and y directions

  img_sx = cv2.Sobel(d_image, cv2.CV_32F, 1, 0, ksize=3)
  img_sy = cv2.Sobel(d_image, cv2.CV_32F, 0, 1, ksize=3)

  img_sx = img_sx/np.max(img_sx)
  img_sy = img_sy/np.max(img_sy)
  
  return(img_sx,img_sy)

def grad_avg_LR(img_sx,img_sy):# calculates gradient average by sector and direction  #xL,xR; yL,yR
    
    global grad_bufferXL
    global grad_bufferXR
    
    img_sx_n=np.sign(img_sx)
    img_sy_n=np.sign(img_sy)
    avg =  {'Lx' : 0 ,'Ly' : 0,
            'Rx' : 0 ,'Ry' : 0,
            'Tx' : 0 ,'Ty' : 0}

    #Lx
    avg['Lx'] = np.mean(img_sx_n[:,:img_sx_n.shape[1]/2]) 
    #Ly
    avg['Ly']= np.mean(img_sy_n[:,:img_sy_n.shape[1]/2])
    #Rx
    avg['Rx'] = np.mean(img_sx_n[:,img_sx_n.shape[1]/2 :])
    #Ry
    avg['Ry'] = np.mean(img_sy_n[:,img_sy_n.shape[1]/2 :])

    
    
  
    
    avg['Lx'],grad_bufferXL= MovAvg(avg['Lx'],grad_bufferXL) #filter 
    avg['Rx'],grad_bufferXR= MovAvg(avg['Rx'],grad_bufferXR) #filter 
    
    #Tx
    avg['Tx'] = (avg['Lx']+avg['Rx'])/2 
    #Ty
    avg['Ty'] = (avg['Ly']+avg['Ry'])/2 
 


    return(avg)

def draw_grid(image_bgr,density=10):#draws grid
  h,w,c =image_bgr.shape
  dy=(int)(h/density)
  dx=(int)(w/density)
  for y in range(0,h,dy):
    cv2.line(image_bgr, (0, y), (w, y), (255, 0, 0), 1, 1)
  for x in range(0,w,dx):
    cv2.line(image_bgr, (x, 0), (x, h), (255, 0, 0), 1, 1)
  return(image_bgr)

def dir_draw_axe(image_bgr,avg_x,avg_y,factor=50,axe = 'x',divergence = False):#draws total gradient and allows to choose axe 
    h,w,c=image_bgr.shape
    av_X = avg_x*factor  
    av_Y = avg_y*factor/2
    start = ((int)(w/2),(int)(h/2))
    endx = ( (int)(w/2) + (int)(av_X)  , (int)(h/2) )
    endy = ( (int)(w/2), (int)(h/2) + (int)(av_Y) )

    if axe == 'x' : 
      if not divergence:
        if np.abs(av_X) > 0.02*w:
          image_bgr_dir = cv2.arrowedLine(image_bgr,start,endx,(0, 255, 0),thickness=2,tipLength = 0.2 )
        else:
          image_bgr_dir = cv2.circle(image_bgr, start, 30,  (255,0,0),thickness=2)
      else:
        image_bgr_dir = cv.putText(image_bgr, "X", ((int)(w/2),(int)(h/2)), cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,255),3)  
        
    
    if axe == 'y' : 
      image_bgr_dir = cv2.arrowedLine(image_bgr,start,endy,(0, 255, 0),thickness=2,tipLength = 0.2 )
    if axe == 'xy' : 
      end = ( (int)(w/2) + (int)(av_X), (int)(h/2) + (int)(av_Y) )
      image_bgr_dir = cv2.arrowedLine(image_bgr,start,endx,(255, 0, 0),thickness=2,tipLength = 0.2 )
      image_bgr_dir = cv2.arrowedLine(image_bgr_dir,start,endy,(0, 255, 0),thickness=2,tipLength = 0.2 )

    # if (np.sqrt(av_X**2+av_Y**2)>0.001*factor):
    #     image_bgr_dir = cv2.arrowedLine(image_bgr,start,end,(255, 0, 0),thickness=2,tipLength = 0.2 )
    # else:
    #     image_bgr_dir = cv2.circle(image_bgr, start, 30,  (255,0,0),thickness=2)
    return(image_bgr_dir)

def dir_draw_LR(image_bgr,avg,factor=50):#draws left and right gradient in x direction ,searchs for divergence and calls dir_draw axe
    h,w,c=image_bgr.shape
    # global grad_bufferXL
    # global grad_bufferXR
    #Obtaining the average gradient by zone
    av_Lx = avg['Lx']*factor  
    av_Rx = avg['Rx']*factor
    av_Ly = avg['Ly']*factor  
    av_Ry = avg['Ry']*factor
    
    av_X = avg['Tx']*factor
    av_Y = avg['Ty']*factor

    # ma_XL,grad_bufferXL= MovAvg(av_XL,grad_bufferXL)
    # ma_XR,grad_bufferXR= MovAvg(av_XR,grad_bufferXR)

    # av_XL = ma_XL
    # av_XR = ma_XR

    #total gradient
    # av_X = (av_XL+av_XR)/2#np.mean(avg[0,:])
    # av_Y = np.mean(avg[1,:])
    
    #computing centers and ends of arroxs
    startxL = ((int)(w/4),(int)(h/2))
    startxR = ((int)(w*3/4),(int)(h/2))
    endxL = ( (int)(w/4) + (int)(av_Lx)  , (int)(h/2) )
    endxR = ( (int)(w*3/4) + (int)(av_Rx)  , (int)(h/2) )
    
    if True:#np.abs(av_XL) > 0.02*w: #draw arrow if module bigger than tolerance LEFT
      image_bgr_dir = cv2.arrowedLine(image_bgr,startxL,endxL,(255, 0, 0),thickness=2,tipLength = 0.2 )
    else: #else draw circle, meaning that we are in the center of the zone LEFT
      image_bgr_dir = cv2.circle(image_bgr, startxL, 30,  (255,0,0),thickness=2)
    if True:#np.abs(av_XR) > 0.02*w: #draw arrow if module bigger than tolerance RIGHT
      image_bgr_dir = cv2.arrowedLine(image_bgr,startxR,endxR,(255, 0, 0),thickness=2,tipLength = 0.2 )
    else: #else draw circle, meaning that we are in the center of the zone RIGTH
      image_bgr_dir = cv2.circle(image_bgr, startxR, 30,  (255,0,0),thickness=2)
   

    #check for divergence between left and right zone
    if np.sign(av_Lx) > 0 and np.sign(av_Rx) < 0 : 
      divergence = False
    elif np.sign(av_Lx) < 0 and np.sign(av_Rx) > 0 :
      divergence = True
    else:
      divergence = False

    #draw total gradient
    image_bgr_dir = dir_draw_axe(image_bgr_dir,av_X,av_Y,divergence = divergence,factor=10)



    return(image_bgr_dir)


def MovAvg(value,grad_buffer): #calculates moving average of given global array
  
  
  amount = np.arange(0,len(grad_buffer),1)**2
  grad_buffer = np.delete( grad_buffer,0)
  grad_buffer = np.append(grad_buffer, value) 
  ma = np.average(grad_buffer, weights=amount) 
  return ma,grad_buffer


def gradient(image):
    #density = 16 #1; 2; 3; 4; 5; 6; 8; 10; 12; 15; 20; 24; 30; 40; 60 COMMON FACTORSS
    
    img_sx,img_sy =  N_sobel(image)
    
   
   
    avg = grad_avg_LR(img_sx,img_sy)
    
    #avgs = grad_avg(img_sx,img_sy,density)

    #avg_x,avg_y = grad_avg_b(img_sx,img_sy)
    
    #image_bgr = draw_grid(image_bgr,density)

    #image_bgr = avg_draw(image_bgr,avgs,100)  #compute with grid
   
    #image_bgr_dir = dir_draw(image_bgr,avgs)

    #image_bgr_dir = dir_draw_axe(image_bgr,avg_x,avg_y) #compute with direct avg over img_sX
    image_bgr = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR) 
    image_bgr_dir = dir_draw_LR(image_bgr,avg) #compute with direct avg over img_sX
    return image_bgr_dir,avg

def process(image):
  colorized_depth = image
  
  # colorized_depth = cv.cvtColor(colorized_depth,cv.COLOR_BGR2GRAY)


  colorized_depth = cv.GaussianBlur(colorized_depth,(9,9),0)
  img_bgr_f,avg = gradient(colorized_depth)
  
  #img_bgr_f = cv.resize(img_bgr_f,(3*img_bgr_f.shape[1], 3*img_bgr_f.shape[0]), interpolation = cv.INTER_CUBIC)
  cv.imshow('grad',img_bgr_f)
  # print("--- %s seconds - Hz-" % (time.time() - start_time),  1/(time.time() - start_time))
  #cv.imshow('grad_o',img_bgr_o)
  # start_time = time.time()
  #print(np.max(colorized_depth))
  cv.waitKey(1)
  return avg

def main():

    # Setup:
    # pipe = rs.pipeline()
    # cfg = rs.config()

    # Start streaming
    # cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    #cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
 
    # Skip 5 first frames to give the Auto-Exposure time to adjust
    # for x in range(5):
    #   pipe.wait_for_frames()

    # colorizer = rs.colorizer(3)#config colorizer type 3=black to white

    # Start streaming 
    # pipe.start(cfg)
    
    #initialize global buffers for moving average 
    global grad_bufferXL
    global grad_bufferXR
    buffer_size=10
    grad_bufferXL =  np.arange(0,buffer_size,1)
    grad_bufferXR =  np.arange(0,buffer_size,1)
    
    global image

    rospy.init_node("Gradient")
    rospy.Subscriber("/d400/depth/image_rect_raw", Image, image_callback)


    start_time = time.time()
    # Store next frameset for later processing:
    
    rospy.spin()
    
    #while True:
        
        # frame_f,original = get_filtered(pipe,filters_init())

        # colorized_depth = np.asanyarray(colorizer.colorize(frame_f).get_data())

        # colorized_depth = image
        # cv2.imshow('test',image)
        # #colorized_depth = cv.cvtColor(colorized_depth,cv.COLOR_BGR2GRAY)

 
        # colorized_depth = cv.GaussianBlur(colorized_depth,(9,9),0)
        # img_bgr_f,avg = gradient(colorized_depth)
        
        # img_bgr_f = cv.resize(img_bgr_f,(3*img_bgr_f.shape[1], 3*img_bgr_f.shape[0]), interpolation = cv.INTER_CUBIC)
        # cv.imshow('grad',img_bgr_f)
        # print("--- %s seconds - Hz-" % (time.time() - start_time),  1/(time.time() - start_time))
        # #cv.imshow('grad_o',img_bgr_o)
        # start_time = time.time()
        # #print(np.max(colorized_depth))
        # cv.waitKey(1)
        
        
        
    # pipe.stop()


if __name__ == '__main__':
   
    # def filters_init():#initializates filters
    #     #creates a dictionary with the filters and configures its parameters  
    #     filters = {}
    #     filters["decimation"]=rs.decimation_filter()

    #     filters["depth_to_disparity"]=rs.disparity_transform(True)
        
    #     filters["spatial"]=rs.spatial_filter()
    #     filters["spatial"].set_option(rs.option.filter_magnitude, 5)
    #     filters["spatial"].set_option(rs.option.filter_smooth_alpha, 1)
    #     filters["spatial"].set_option(rs.option.filter_smooth_delta, 50)
    #     filters["spatial"].set_option(rs.option.holes_fill, 3)
        
    #     filters["disparity_to_depth"]=rs.disparity_transform(False)
        
    #     filters["hole_filling"]=rs.hole_filling_filter()

    #     return filters

    # def get_filtered(pipe,filters): #gets filtered and original frames from camera
    #     #recieves the pipe to get the trqnsmision and a dictionary with the corresponding filters
    #     #and implements them
    #     frameset = pipe.wait_for_frames()
    #     frame = frameset.get_depth_frame()
    #     frame_original=frame
    #     #I can't excecute the following with a for because dictionaries aren't ordered
    #     frame = filters["decimation"].process(frame)
    #     frame = filters["depth_to_disparity"].process(frame)
    #     frame = filters["spatial"].process(frame)
    #     #frame = temporal.process(frame)
    #     frame = filters["disparity_to_depth"].process(frame)
    #     frame = filters["hole_filling"].process(frame)
        
    #     return frame,frame_original

    main()