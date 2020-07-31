import numpy as np                        # fundamental package for scientific computing
import matplotlib.pyplot as plt           # 2D plotting library producing publication quality figures
import pyrealsense2 as rs                 # Intel RealSense cross-platform open-source API
import cv2 as cv,cv2
import time




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
    
    return frame,frame_original,frameset


def get_factor(gray_depth,frame_f,value=200): #returns conversion factor between real depth and grayscale and the value usedto calculate it (you can specify it), by default is 200 because of the noise

    value_indexes = np.where(gray_depth == value)
    listOfCoordinates= list(zip(value_indexes[0], value_indexes[1]))
    while not listOfCoordinates:
        value=value-1
        value_indexes = np.where(gray_depth == value)
        listOfCoordinates= list(zip(value_indexes[0], value_indexes[1]))
        
    point= listOfCoordinates[0]
    y_p = point[0]
    x_p = point[1]
    point_depth = frame_f.as_depth_frame().get_distance((int(x_p)), (int(y_p)))
    factor = point_depth/value

    return(factor,value)

def factor_error_m(frame_f,factor):
    R_depth = frame_f.as_depth_frame().get_distance((int(w/2)), (int(h/2)))
    E_depth = gray_depth[(int(h/2)), (int(w/2))]*factor
    error = R_depth-E_depth
    return error

def main():

    # Setup:
    pipe = rs.pipeline()
    cfg = rs.config()

    # Start streaming
    cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    #cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    colorizer = rs.colorizer(3)#config colorizer type 3=black to white

    # Start streaming 
    pipe.start(cfg)
    
   
    # Store next frameset for later processing:
    while True:
        
        frame_f,original,frameset = get_filtered(pipe,filters_init())

        colorized_depth = np.asanyarray(colorizer.colorize(frame_f).get_data())
        colorized_depth = cv.GaussianBlur(colorized_depth,(9,9),0)

        gray_depth = cv.cvtColor(colorized_depth,cv.COLOR_BGR2GRAY)
        colorized_depth_o = np.asanyarray(colorizer.colorize(original).get_data())
        
        h,w = gray_depth.shape #240x320
        
        
        factor,used_val =get_factor(gray_depth,frame_f)

        #colorized_depth = cv2.circle(colorized_depth, (x_max,y_max), 10,  (255,0,0),thickness=2)   
        
        test_point=(300,100)
        depth_intrin = frame_f.profile.as_video_stream_profile().intrinsics
        R_depth = frame_f.as_depth_frame().get_distance((int(300)), (int(100)))
        
        
        depth_point = rs.rs2_deproject_pixel_to_point(depth_intrin,[0,0], R_depth)#intrinsics, pixel coord (y,x) and depth coord
       
        
        portal_center = [0,0,1]
        portal_topL = [-0.25,-0.1,1]
        portal_bottomR = [0.25,0.1,1]
        
        uvtl = rs.rs2_project_point_to_pixel(depth_intrin,portal_topL)
        uvbr = rs.rs2_project_point_to_pixel(depth_intrin,portal_bottomR)
        center = rs.rs2_project_point_to_pixel(depth_intrin,portal_center)

        uvtl = [int(round(x)) for x in uvtl]
        uvbr = [int(round(x)) for x in uvbr]
        center = [int(round(x)) for x in center]
        print(uvtl,center,uvbr)
        print(uvtl[0])
        #point = rs.rs2_project_point_to_pixel(depth_intrin,depth_point)
        
    

        #colorized_depth = cv2.circle(colorized_depth, (300,100), 10,  (255,0,0),thickness=2)   


        colorized_depth = cv.rectangle(colorized_depth , (uvtl[0],uvtl[1]), (uvbr[0],uvbr[1]), (0,255,0),3)
        colorized_depth = cv.resize(colorized_depth,(2*colorized_depth.shape[1], 2*colorized_depth.shape[0]), interpolation = cv.INTER_CUBIC)
        cv.imshow("normal",colorized_depth)
        

        # ret,binary_1 = cv.threshold(gray_depth,0.25/factor,255,cv.THRESH_BINARY)
        # cv.imshow("binary",binary_1)

        
        cv.waitKey(1)

        
    pipe.stop()


if __name__ == '__main__':
    main()