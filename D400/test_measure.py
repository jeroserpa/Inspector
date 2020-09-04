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

def m_to_px(depth_intrin,size,depth,axe=0):
    
    if axe == 0:
        start = [-size/2,0,depth]
        end = [size/2,0,depth]
        start_px = rs.rs2_project_point_to_pixel(depth_intrin,start)
        end_px = rs.rs2_project_point_to_pixel(depth_intrin,end)
        return (end_px[0]-start_px[0])
    else:
        start = [0,-size/2,depth]
        end = [0,size/2,depth]
        start_px = rs.rs2_project_point_to_pixel(depth_intrin,start)
        end_px = rs.rs2_project_point_to_pixel(depth_intrin,end)
        return (end_px[1]-start_px[1])

def find_free_path(image,depth_intrin,drone_w,drone_h,layer_depth,step=25):
    layer = image.copy()
    h,w = layer.shape
    rw= int(m_to_px(depth_intrin,drone_w,layer_depth,0))
    rh= int(m_to_px(depth_intrin,drone_h,layer_depth,1))
    paths = np.zeros([100,2])
    i=0
    for y in range(0,h-rh,step):
        for x in range(0,w-rw,step):
            sector = layer[y:y+rh,x:x+rw]
            if np.all(sector>10):  
                layer[y:y+rh,x:x+rw] = 175
                paths[i,...]=[int(x+rw/2),int(y+rh/2)] 
    layer[layer<10] = 255
    layer[layer<200] = 0
    contours, hier = cv2.findContours(np.abs(layer-255), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    return contours,paths

def get_bin_layer(gray_depth,factor,depth):
    ret,thresh = cv.threshold(gray_depth,depth/factor,255,cv.THRESH_BINARY)
    kernel=np.ones([3,3])
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel, iterations=10)
    return thresh




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
        if factor != 0 :

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
            # print(uvtl,center,uvbr)
            # print(uvtl[0])
            #point = rs.rs2_project_point_to_pixel(depth_intrin,depth_point)
            


            drone_w = 0.50
            drone_h = 0.30
            
                            
            thresh1 = get_bin_layer(gray_depth,factor,1)
            thresh2 = get_bin_layer(gray_depth,factor,2)
            thresh3 = get_bin_layer(gray_depth,factor,3)
            thresh4 = get_bin_layer(gray_depth,factor,4)
            
            contours1,paths1 = find_free_path(thresh1,depth_intrin,drone_w,drone_h,1)
            contours2,paths2 = find_free_path(thresh2,depth_intrin,drone_w,drone_h,2)
            contours3,paths3 = find_free_path(thresh3,depth_intrin,drone_w,drone_h,3)
            contours4,paths4 = find_free_path(thresh4,depth_intrin,drone_w,drone_h,4)
            
            
            
            cv2.drawContours(colorized_depth, contours1, -1, (0, 255, 0), 1)
            cv2.drawContours(colorized_depth, contours2, -1, (85, 170, 0), 1)
            cv2.drawContours(colorized_depth, contours3, -1, (170, 85, 0), 1)
            cv2.drawContours(colorized_depth, contours4, -1, (255, 0, 0), 1)
            # cv2.drawContours(thresh, contours, -1, (0, 255, 0), 1)
            
            # colorized_depth = cv.rectangle(colorized_depth , (uvtl[0],uvtl[1]), (uvbr[0],uvbr[1]), (0,255,0),3)
            colorized_depth = cv.resize(colorized_depth,(2*colorized_depth.shape[1], 2*colorized_depth.shape[0]), interpolation = cv.INTER_CUBIC)
            cv.imshow("colorized",colorized_depth)
            
            
        

        # cv.imshow("binary",binary_1)

        
        cv.waitKey(1)

        
    pipe.stop()


if __name__ == '__main__':
    main()