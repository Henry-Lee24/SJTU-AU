import cv2
import numpy as np
import matplotlib.pyplot as plt
def historgram(image):
 # Make a gray histogram of a given image, intensity [0,256]
 plt.hist(image.flatten(), 256, [0, 256], color = 'b') # plot the historgram
 plt.xlim([0, 256]) # set x
 plt.show()
 

def gaussian_noise(image, mean=0, var=0.001):
 # add gasuss noise
 image = np.array(image/255, dtype= float)
 noise = np.random.normal(mean, var ** 0.5, image.shape)
 out = image + noise
 if out. min() < 0:
  low_clip = -1.
 else:
  low_clip = 0.
 out = np.clip(out, low_clip, 1.0)
 out = np.uint8(out*255)
 return out

if __name__ == '__main__':
 img = cv2.imread('./cameraman.jpg', 0)
 
 # Add gaussian noise
 gauss1 = gaussian_noise(img,0,0.001)
 #historgram(gauss1)
 gauss2 = gaussian_noise(img,0,0.005)
 #historgram(gauss2)
 """ plt.subplot(321)
 plt.imshow(img, cmap='gray')
 plt.title('original image')
 plt.subplot(322)
 historgram(img)
 plt.subplot(323)
 plt.imshow(gauss1, cmap='gray')
 plt.title('var=0.001')
 plt.subplot(324)
 historgram(gauss1)
 plt.subplot(325)
 plt.imshow(gauss2, cmap='gray')
 plt.title('var=0.005')
 plt.subplot(326)
 historgram(gauss2) """
 #Remove gassuain noise by gaussian blur (for image gauss2)
 dst1=cv2.GaussianBlur(gauss2,(7,7),3)
 dst2=cv2.GaussianBlur(gauss2,(9,9),3)
 dst3=cv2.GaussianBlur(gauss2,(11,11),3)
 plt.subplot(131)
 plt.imshow(dst1,cmap='gray')
 plt.title('ksize = 7*7')
 plt.subplot(132)
 plt.imshow(dst2,cmap='gray')
 plt.title('ksize = 9*9')
 plt.subplot(133)
 plt.imshow(dst3,cmap='gray')
 plt.title('ksize = 11*11')
 plt.show()
 #cv2.imwrite('./dst.png',dst)
