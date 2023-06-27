#!/usr/bin/env python
 
# from this import d
import rospy
import random
import numpy as np
from gazebo_msgs.msg import ModelState
# from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
from scipy import linalg as lnr
 
class Driver(object):
    # constructor
    def __init__(self):
        self.time_save = 0
        self.name = 'cylinderRobot'
        self.mass = 10
        ## ===================================== Edit bellow =====================================
        ## ==========================================================================   vvvvvvvvvv

        # Define publisher object.  Publish the state of robot to topic "/gazebo/set_model_state"
        #   message type is ModelState
        self.pub = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=10)
        # Define subscriber object. Subscrip the signal from  topic  "/robot/control" sent by teleop
        #   message type is Twist
        #   set the callback function as self.callback_control
        rospy.Subscriber("/robot/control",Twist, self.callback_control)

        # member variable saving system state.
        self.state = np.zeros([4,1])    
        # state = [ vx; px; vy; py ]
        #   dx/dt = Ax(t) + Bu(t) + w(t), 
        #   cov[w(t), w(t)] = Sigma_w

        # Define matrix of continuous system:  A, B (by numpy).
        self.A = np.identity(4)
        self.B = np.zeros([4,2])
	

        ## ==========================================================================  ^^^^^^^^^^
        ## ===================================== Edit above =====================================
        self.Sigma_w = np.eye(4)*0.00001
 
    def callback_control(self, twist):
        if self.time_save == 0:
            self.time_save = rospy.get_time()
        else:
            dt = rospy.get_time() - self.time_save                              # Sampling interval (s)
            self.time_save = rospy.get_time()
            u = np.zeros([2,1])
            u[0] = twist.linear.x                                               # Robot power input
            u[1] = twist.linear.y
	    if u[0] ==0 and self.state[1,0]==0 and self.state[3,0]==0:
		    return 0
	    else:
                self.state = self.forward_dynamics(u, dt)
                self.sendStateMsg()

    def forward_dynamics(self, u, dt):                                          # Mass kinetic functions

        Atilde, Btilde, Sigma_w_tilde = self._discretization_Func(dt)           # Matrices A, B, W

        w = np.random.multivariate_normal(np.zeros([4]), Sigma_w_tilde).reshape([4, 1])

        x = Atilde.dot(self.state) + Btilde.dot(u) + w                  # Dynamics model

        return x                                                        # Return status sequence

    def _discretization_Func(self, dt):
        ## ===================================== Edit bellow =====================================
        ## ==========================================================================   vvvvvvvvvv


       # Please implementation the discretization function here
        Atilde = np.array([
            [1, 0, 0, 0],
            [dt, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, dt, 1]
        ])
        Btilde = np.array([
            [dt / self.mass, 0],
            [dt * dt / 2 / self.mass, 0],
            [0, dt / self.mass],
            [0, dt * dt / 2 / self.mass]
        ])
        q1 = self.Sigma_w[0, 0]
        q2 = self.Sigma_w[1, 1]
        q3 = self.Sigma_w[2, 2]
        q4 = self.Sigma_w[3, 3]
        Sigma_w_tilde = np.array([
            [dt * q1, dt * dt / 2 * q1, 0, 0],
            [dt * dt / 2 * q1, (dt * q2) + (dt * dt * dt / 3 * q1), 0, 0],
            [0, 0, dt * q3, dt * dt / 2 * q3],
            [0, 0, dt * dt / 2 * q3, (dt * q4) + (dt * dt * dt / 3 * q3)],
        ])

        return Atilde, Btilde, Sigma_w_tilde

    def sendStateMsg(self):
        msg = ModelState()
        msg.model_name = self.name
        msg.pose.position.x = self.state[1]
        msg.pose.position.y = self.state[3]
        self.pub.publish(msg)
 
       
       
 
if __name__ == '__main__':
    try:
        rospy.init_node('driver', anonymous=True)
        driver = Driver()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
