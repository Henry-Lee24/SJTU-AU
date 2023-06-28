import cv2
import matplotlib.pyplot as plt
import numpy as np
def historgram(image):
 #print(cdf)
 plt.hist(image.flatten(), 256, [0, 256], color = 'r') # plot the historgram
 plt.xlim([0, 256]) # set x
 plt.show()
 

if __name__ == '__main__':
  img1 = cv2.imread('./cameraman.jpg', 0) # read the gray value of the image
  historgram(img1) # draw the gray value of the original image
  img2 = cv2.equalizeHist(img1) #make equalization
  historgram(img2)
  cv2.imwrite('./OriginImage.png',img1)
  cv2.imwrite('./NewImage.png',img2)
  cv2.waitKey(0)
  cv2.destroyAllWindows()


