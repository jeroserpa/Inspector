import numpy as np                        # fundamental package for scientific computing
import matplotlib.pyplot as plt           # 2D plotting library producing publication quality figures
import pyrealsense2 as rs                 # Intel RealSense cross-platform open-source API
import cv2 as cv,cv2
import time
import openspace
import depthgradient as gd

print("Environment Ready")

def filters_init():#initializates filters
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

def get_filtered(pipe,filters): #gets filtered and original frames from camera
    #recieves the pipe to get the trqnsmision and a dictionary with the corresponding filters
    #and implements them
    frameset = pipe.wait_for_frames()
    frame = frameset.get_depth_frame()
    frame_original=frame
    #I can't excecute the following with a for because dictionaries aren't ordered
    frame = filters["decimation"].process(frame)
    frame = filters["depth_to_disparity"].process(frame)
    frame = filters["spatial"].process(frame)
    #frame = temporal.process(frame)
    frame = filters["disparity_to_depth"].process(frame)
    frame = filters["hole_filling"].process(frame)
    
    return frame,frame_original

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
    
    #initialize global buffers for moving average 
    global grad_bufferXL
    global grad_bufferXR
    buffer_size=10
    grad_bufferXL =  np.arange(0,buffer_size,1)
    grad_bufferXR =  np.arange(0,buffer_size,1)
    
    start_time = time.time()
    # Store next frameset for later processing:
    while True:
        
        frame_f,original = get_filtered(pipe,filters_init())

        colorized_depth = np.asanyarray(colorizer.colorize(frame_f).get_data())

        colorized_depth = cv.cvtColor(colorized_depth,cv.COLOR_BGR2GRAY)

 
        colorized_depth = cv.GaussianBlur(colorized_depth,(9,9),0)
        img_bgr_f = gd.gradient(colorized_depth)
        img_bgr_f = cv.resize(img_bgr_f,(3*img_bgr_f.shape[1], 3*img_bgr_f.shape[0]), interpolation = cv.INTER_CUBIC)
        cv.imshow('grad',img_bgr_f)
        print("--- %s seconds - Hz-" % (time.time() - start_time),  1/(time.time() - start_time))
        #cv.imshow('grad_o',img_bgr_o)
        start_time = time.time()
        #print(np.max(colorized_depth))
        cv.waitKey(1)
        
        
        
    pipe.stop()


if __name__ == '__main__':
    main()