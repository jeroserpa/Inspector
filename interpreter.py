import numpy as np                        # fundamental package for scientific computing
import matplotlib.pyplot as plt           # 2D plotting library producing publication quality figures
import pyrealsense2 as rs                 # Intel RealSense cross-platform open-source API
import cv2 as cv,cv2
import time

print("Environment Ready")

def filters_init():
    #creates a dictionary with the filters and configures its parameters
    filters = {}
    filters["decimation"]=rs.decimation_filter()

    filters["depth_to_disparity"]=rs.disparity_transform(True)
    
    filters["spatial"]=rs.spatial_filter()
    filters["spatial"].set_option(rs.option.filter_magnitude, 5)
    filters["spatial"].set_option(rs.option.filter_smooth_alpha, 1)
    filters["spatial"].set_option(rs.option.filter_smooth_delta, 50)
    filters["spatial"].set_option(rs.option.holes_fill, 3)
    
    filters["disparity_to_depth"]=rs.disparity_transform(False)
    
    filters["hole_filling"]=rs.hole_filling_filter()

    return filters


def get_filtered(pipe,filters):
    #recieves the pipe to get the trqnsmision and a dictionary with the corresponding filters
    #and implements them
    frameset = pipe.wait_for_frames()
    frame = frameset.get_depth_frame()

    #I can't excecute the following with a for because dictionaries aren't ordered
    frame = filters["decimation"].process(frame)
    frame = filters["depth_to_disparity"].process(frame)
    frame = filters["spatial"].process(frame)
    #frame = temporal.process(frame)
    frame = filters["disparity_to_depth"].process(frame)
    frame = filters["hole_filling"].process(frame)
    
    return frame


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



def main():

    # Setup:
    pipe = rs.pipeline()
    cfg = rs.config()

    # Start streaming
    cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    #cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
 
    # Skip 5 first frames to give the Auto-Exposure time to adjust
    # for x in range(5):
    #   pipe.wait_for_frames()

    colorizer = rs.colorizer(3)#config colorizer type 3=black to white

    # Start streaming 
    pipe.start(cfg)
    
    # Store next frameset for later processing:

    while True:
        start_time = time.time()
        frame_f = get_filtered(pipe,filters_init())

        colorized_depth = np.asanyarray(colorizer.colorize(frame_f).get_data())
        # cv.imshow("normal",colorized_depth)
        
        # for y in range(480/2):
        #     for x in range(640/2):
        #         dist_d[(int(y/2)),(int(x/2))] = frame.as_depth_frame().get_distance((int(x)), (int(y)))

        


        cv.imshow('filtered',colorized_depth)
        colorized_depth = cv.cvtColor(colorized_depth,cv.COLOR_BGR2GRAY)
        img_bgr=gradient(colorized_depth)

        cv.imshow('grad',img_bgr)

        print(np.max(colorized_depth))
        cv.waitKey(1)
        #print("--- %s seconds - Hz-" % (time.time() - start_time),  1/(time.time() - start_time))
        
        
    pipe.stop()


if __name__ == '__main__':
    main()