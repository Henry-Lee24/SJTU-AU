import tensorflow.compat.v1 as tf
tf.disable_v2_behavior()
import matplotlib.pyplot as plt
import numpy as np
input = np.array([[0,1,0,1,0,1],[0,1,0,1,0,1],[0,1,0,1,0,1],[0,1,0,1,0,1],[0,1,0,1,0,1],[0,1,0,1,0,1]])
input = input.reshape([1,6,6,1]) #因为conv2d的参数都是四维的，因此要reshape成四维
kernel = np.array([[1,1,1],[1,1,1],[1,1,1]])
#kernel = kernel/3.0
kernel = kernel.reshape([3,3,1,1]) #kernel也要reshape
print(input.shape,kernel.shape) #(1, 6, 6, 1) (3, 3, 1, 1)

x = tf.placeholder(tf.float32,[1,6,6,1])
k = tf.placeholder(tf.float32,[3,3,1,1])
output = tf.nn.conv2d(x,k,strides=[1,1,1,1],padding='SAME')


with tf.Session() as sess:
    y = sess.run(output,feed_dict={x:input,k:kernel})
    print(y.shape) #(1,6,6,1)
    print(y) #因为y有四维，输出太长了，我就只写一下中间两维的结果（6*6部分）：
    #[[0,0,1/3,4/3,2,4/3],                 [[1/3,1/3,4/3,1/3,4/3,1/3], 
    # [0,0,1,2,3,2],                        [1,1,2,1,2,1],
    # [0,0,1,2,3,2],                        [1,1,2,1,2,1],
    # [0,0,1,2,3,2],                        [1,1,2,1,2,1],
    # [0,0,1,2,3,2],                        [1,1,2,1,2,1],
    # [0,0,1/3,4/3,2,4/3]]                  [1/3,1/3,4/3,1/3,4/3,1/3]]

#a = np.array([0,0,1/3,4/3,2,4/3,0,0,1,2,3,2,0,0,1,2,3,2,0,0,1,2,3,2,0,0,1,2,3,2,0,0,1/3,4/3,2,4/3])
a = np.array([1/3,1/3,4/3,1/3,4/3,1/3,1/3,1/3,4/3,1/3,4/3,1/3,1,1,2,1,2,1,1,1,2,1,2,1,1,1,2,1,2,1,1,1,2,1,2,1])
a1 = a*9
b = a.flatten()
plt.hist(b,color='b')
plt.show()