import numpy as np                        # fundamental package for scientific computing
import matplotlib.pyplot as plt           # 2D plotting library producing publication quality figures
import pyrealsense2 as rs                 # Intel RealSense cross-platform open-source API
import cv2 as cv,cv2
import time

print("Environment Ready")

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

#initialize and config filters
decimation = rs.decimation_filter()
spatial = rs.spatial_filter()
spatial.set_option(rs.option.filter_magnitude, 5)
spatial.set_option(rs.option.filter_smooth_alpha, 1)
spatial.set_option(rs.option.filter_smooth_delta, 50)
spatial.set_option(rs.option.holes_fill, 3)
#temporal = rs.temporal_filter()
hole_filling = rs.hole_filling_filter()
depth_to_disparity = rs.disparity_transform(True)
disparity_to_depth = rs.disparity_transform(False)


# Start streaming 
pipe.start(cfg)

dist_d = np.zeros([480/2,640/2])
frames = []
stack = 1
# Store next frameset for later processing:
while True:
    start_time = time.time()
    
    
    frameset = pipe.wait_for_frames()
    frames.append(frameset.get_depth_frame())

    colorized_depth = np.asanyarray(colorizer.colorize(frames[stack-1]).get_data())
    cv.imshow("normal",colorized_depth)
    #print("start filter")
    for x in range(stack):
        frame = frames[x]
        frame = decimation.process(frame)
        frame = depth_to_disparity.process(frame)
        frame = spatial.process(frame)
        #frame = temporal.process(frame)
        frame = disparity_to_depth.process(frame)
        frame = hole_filling.process(frame)
    
    colorized_depth = np.asanyarray(colorizer.colorize(frame).get_data())
    # for y in range(480/2):
    #     for x in range(640/2):
    #         dist_d[(int(y/2)),(int(x/2))] = frame.as_depth_frame().get_distance((int(x)), (int(y)))

    # dist_d_im = dist_d/np.max(dist_d)
    # cv.imshow("d",dist_d_im)
    cv.imshow('filt',colorized_depth)
    cv.waitKey(1)
    print("--- %s seconds - Hz-" % (time.time() - start_time),  1/(time.time() - start_time))
    frames[:] = []
    #plt.show()
pipe.stop()