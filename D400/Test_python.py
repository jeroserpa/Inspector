import numpy as np
import cv2 
import sys
print(cv2.__version__)
img_g = cv2.imread('PNG_depth_Depth_1594921101977.93041992187500.png',cv2.IMREAD_COLOR)

cv2.imwrite('gray_image_original.png', img_g)
#im_color = cv2.applyColorMap(img_g, cv2.COLORMAP_JET)
#cv2.imwrite('colormap.png', im_color)





cv2.imwrite('gray_image_reconstructed.png', img_g)

#cv2.imshow('imag',J)
#cv2.waitKey()

print(sys.version)
