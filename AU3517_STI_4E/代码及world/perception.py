#!/usr/bin/env python

import rospy
import rospkg
from scipy import linalg as lnr
from matplotlib import pyplot as plt
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelState
import os
import sys

# import the Kalman filter we finished last week
# from KalmanFilter import KalmanFilter

class KalmanFilter(object):                                                 # The class definition: Kalman filter. 
    # initialization the kalman filter. 
    #   x'(t) = Ax(t) + Bu(t) + w(t)
    #   y(t) = Cx(t) + v(t)
    #   x(0) ~ N(x_0, P_0)
    def __init__(self, mass, C, Sigma_w, Sigma_v, x_0, P_0):                # constructor
        self.mass = mass                                                    # Mass of the mass dynamics model
        self.C = C                                                          # Observation matrix C
    
        self.n = 4                                                          # The state has 4 dimensions (vx, px, vy, py)
        self.m = 2                                                          # Control has 2 dimensions (fx, fy)

        self.Sigma_w = Sigma_w                                              # Covariance matrix of continuous time course noise (diagonal array)
        self.Sigma_v = Sigma_v                                              # Covariance matrix of the observed noise
        
        self.t = 0                                                          # Record the current time
        self.x = x_0                                                        # Mathematical expectation of the initial state of the system
        self.P = P_0                                                        # Covariance matrix of the initial state of the system
        self.u = np.zeros([self.m, 1])                                      # Initialize the control signal to 0

    # Given duration dt, return the discretization of A, B, Sigma_w. Just like what we do last week.
    def _discretization_Func(self, dt):                                     # Compute the discretized system matrix and calculate At, Bt, sum of w according to the sampling interval
        Atilde = np.array([
            [1, 0,  0,  0],                                                    
            [dt,1,  0,  0],
            [0, 0,  1,  0],
            [0, 0,  dt, 1]
        ])
        Btilde = np.array([
            [dt/self.mass,      0],
            [dt*dt/2/self.mass, 0],
            [0,      dt/self.mass],
            [0, dt*dt/2/self.mass]
        ])
        q1 = self.Sigma_w[0,0]
        q2 = self.Sigma_w[1,1]
        q3 = self.Sigma_w[2,2]
        q4 = self.Sigma_w[3,3]
        Sigma_w_tilde = np.array([
            [dt*q1,         dt*dt/2*q1,                 0,          0],
            [dt*dt/2*q1,    (dt*q2)+(dt*dt*dt/3*q1),    0,          0],
            [0,             0,                          dt*q3,      dt*dt/2*q3],
            [0,             0,                          dt*dt/2*q3, (dt*q4)+(dt*dt*dt/3*q3)],
        ])

        return Atilde, Btilde, Sigma_w_tilde

    ################################################################################################
    ## ================================ Edit below here ================================ vvvvvvvvvvv
    # predict step
    def _predict_Step(self, ctrl_time):
        dt = ctrl_time - self.t # calculate the time interval
        self.t = ctrl_time # update the current time

        At, Bt, Sigma = self._discretization_Func(dt) # get the discrete system matrices

        self.x = At.dot(self.x) + Bt.dot(self.u) # predict the state using the system equation
        self.P = At.dot(self.P).dot(At.T) + Sigma # predict the covariance using the error equation


    # correction step
    def _correction_Step(self, y):                                      
        innovation = self.P.dot(self.C.T).dot(
            np.linalg.inv(self.C.dot(self.P).dot(self.C.T) + self.Sigma_v))  # calculate the Kalman gain
        self.x =  self.x + innovation.dot(y - self.C.dot(self.x)) # correct the state using the measurement residual
        self.P = (np.eye(4) - innovation.dot(self.C)).dot(self.P) # correct the covariance using the Kalman gain


    # when getting the control signal, execution the predict step, update the control signal
    def control_moment(self, u_new, time_now):                          # Control moment
        self._predict_Step(time_now)  # execute the predict step with current time
        self.u = u_new # update the control signal  

    # when getting the observe info, execution the predict step, and then execution the correction step
    def observe_moment(self, y_new, time_now):                          # The observation time
        self._predict_Step(time_now)  # execute the predict step with current time
        self._correction_Step(y_new)  # execute the correction step with new observation
        

    ## ==========================================================================  ^^^^^^^^^^
    ## ===================================== Edit above =====================================
class Localization(object):
    def __init__(self):
        # config the subscribe information
        rospy.Subscriber('/robot/control', Twist, self.callback_control)
        rospy.Subscriber('/robot/observe', LaserScan, self.callback_observe)
        rospy.Subscriber('gazebo/set_model_state', ModelState, self.callback_state)
        self.pub = rospy.Publisher("/robot/esti_model_state", ModelState, queue_size=10)
        # catch Ctrl+C. When you press Ctrl+C, call self.visualzation()
        rospy.on_shutdown(self.visualization)

        # initialize Kalman filter. 
        self.kf = KalmanFilter(
            mass = 10, 
            C = np.array([
                [0, 1, 0, 0],
                [0, 0, 0, 1]
            ]),
            Sigma_w = np.eye(4)*0.00001,
            Sigma_v = np.array([[0.02**2, 0],[0, 0.02**2]]),
            x_0 = np.zeros([4,1]),
            P_0 = np.eye(4)/1000
            )

        # list to save data for visualization
        self.x_esti_save = []
        self.x_esti_time = []
        self.x_true_save = []
        self.x_true_time = []
        self.p_obsv_save = []
        self.p_obsv_time = []

    ################################################################################################
    ## ================================ Edit below here ================================ vvvvvvvvvvv
    def callback_control(self, twist):
        # extract control signal from message
        u = np.zeros([2,1])
        u[0] = twist.linear.x
        u[1] = twist.linear.y
        current_time = rospy.get_time()

        # call control moment function in Kalman filter
        self.kf.control_moment(u,current_time)
        

        # save data for visualization
        self.x_esti_save.append(self.kf.x)
        self.x_esti_time.append(current_time)

    def callback_observe(self, laserscan):
        # extract observe signal from message
        y = np.zeros([2,1])      
        # y[0,0] = 5-laserscan.ranges[0]
        # y[1,0] = 5-laserscan.ranges[1]
        #Laser measurement of the distance between the robot and the wall in two directions
        x_init = laserscan.ranges[0]
        y_init = laserscan.ranges[1]
        #print("x: ", x_init,"y:", y_init)
        #Calculate the current position of the robot
        y[0,0] = 185/17.0-9.0/17*(2*x_init + y_init)
        y[1,0] = 2*( y[0,0] -5 + x_init) -5
        #print("x_true: ", y[0,0],"y_true:", y[1,0])

        current_time = rospy.get_time()       
        # call observe moment function in Kalman filter
        self.kf.observe_moment(y,current_time)       
        # save data for visualzation
        self.x_esti_save.append(self.kf.x)
        self.x_esti_time.append(current_time)
        self.p_obsv_save.append(y)
        self.p_obsv_time.append(current_time)
        # send estimated x to controller
        self.sendStateMsg()
    ## ==========================================================================  ^^^^^^^^^^
    ## ===================================== Edit above =====================================

    # restore the true state of robot for visualization. You CAN NOT get them in real world.
    def callback_state(self, state):
        current_time = rospy.get_time()
        x = np.zeros([4,1])
        x[0,0] = state.twist.linear.x
        x[1,0] = state.pose.position.x
        x[2,0] = state.twist.linear.y
        x[3,0] = state.pose.position.y
        self.x_true_save.append(x)
        self.x_true_time.append(current_time)

    def sendStateMsg(self):
        msg = ModelState()
        # msg.model_name = se   lf.name
        msg.pose.position.x = self.kf.x[1]
        msg.pose.position.y = self.kf.x[3]
        msg.twist.linear.x = self.kf.x[0]# velocity
        msg.twist.linear.y = self.kf.x[2]
        self.pub.publish(msg)

    # visualzation
    def visualization(self):
        print("Visualizing......")
        t_esti = np.array(self.x_esti_time)
        x_esti = np.concatenate(self.x_esti_save, axis=1)
        
        p_obsv = np.concatenate(self.p_obsv_save, axis=1)
        t_obsv = np.array(self.p_obsv_time)

        t_true = np.array(self.x_true_time)
        x_true = np.concatenate(self.x_true_save, axis=1)

        fig_x = plt.figure(figsize=(16,9))
        ax = fig_x.subplots(2,2)

        ax[0,0].plot(t_esti, x_esti[1,:].T, label = "esti")
        ax[0,0].plot(t_true, x_true[1,:].T, label = "true")
        ax[0,0].legend(bbox_to_anchor = (0.85,1), loc='upper left')
        ax[0,0].set_title('px')

        ax[1,0].plot(t_esti, x_esti[3,:].T, label = "esti")
        ax[1,0].plot(t_true, x_true[3,:].T, label = "true")
        ax[1,0].legend(bbox_to_anchor = (0.85,1), loc='upper left')
        ax[1,0].set_title('py')

        ax[0,1].plot(x_esti[1,:].T, x_esti[3,:].T, label = "esti")
        ax[0,1].plot(x_true[1,:].T, x_true[3,:].T, label = "true")
        ax[0,1].legend(bbox_to_anchor = (0.1,1), loc='upper left')
        ax[0,1].set_title('trace: esti with truth')

        ax[1,1].plot(x_esti[1,:].T, x_esti[3,:].T, label = 'esti')
        ax[1,1].plot(p_obsv[0,:].T, p_obsv[1,:].T, label = 'obsv')
        ax[1,1].legend(bbox_to_anchor = (0.1,1), loc='upper left')
        ax[1,1].set_title('trace: esti with observation')



        fig_path = rospkg.RosPack().get_path('cylinder_robot')+"/"
        fig_x.savefig(fig_path+'fig_x.png', dpi=120)
        print("Visualization Complete.")



if __name__ == '__main__':
    try:
        rospy.init_node('perception', anonymous=True)
        obs = Localization()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
