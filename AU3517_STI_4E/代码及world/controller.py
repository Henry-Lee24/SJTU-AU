#!/usr/bin/env python

import rospy
import sys
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelState
import numpy as np
import math
global x, y, vx, vy,last_x, last_y
global time_now,last_time
x, y, vx, vy,last_x, last_y = 5.1,5.1,0,0,0,0
time_now,last_time = 0,0
class PID_Controller:
 
    def __init__(self,kp,ki,kd,output_min,output_max):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.error = 0 
        self.last_error = 0 
        self.error_sum = 0
        self.error_diff = 0
        self.output_min = output_min
        self.output_max = output_max
        self.output = 0

    def constrain(self, output):
        if output > self.output_max:
            output = self.output_max
        elif output < self.output_min:
            output = self.output_min
        else:
            output = output
        return output

    def get_output(self, error):
        self.error = error
        self.error_sum += self.error
        self.error_diff = self.error - self.last_error
        self.last_error = self.error

        output = self.kp * self.error + self.ki * self.error_sum + self.kd * self.error_diff
        self.output = self.constrain(output)
        return self.output

class controller(object):
    def __init__(self):

        self.pub = rospy.Publisher('/robot/control', Twist, queue_size = 10)
        rospy.Subscriber("/robot/esti_model_state",ModelState, self.callback)

    def callback(self, ModelState):
        global x, y, vx, vy,last_x, last_y, vy_sum
        global time_now,last_time
        last_x = x
        last_y = y
        last_time = time_now
        x = ModelState.pose.position.x
        y = ModelState.pose.position.y
        get_time = rospy.Time.now()
        time_now = get_time.to_sec()
        delta_time = time_now - last_time
        # vx = (x- last_x)/delta_time
        # vy = (y- last_y)/delta_time         
        vx = ModelState.twist.linear.x
        vy = ModelState.twist.linear.y
    def velocity_publisher(self):
        #Set global variables
        global x, y, vx, vy,last_x, last_y
        global time_now,last_time
        rate = rospy.Rate(10)
        state = 0
        twist = Twist()
        #Set the error of target position and acceptable distance from the target
        L, H =4, 3
        move_er_threshold = 0.02
        rotate_er_threshold = 1e-3 
        #Set the parameters of the PID controller (four in total, corresponding to 
        #the x-direction and obtaining the controller move_controller_x for speed from the position,
        #controller move for force F obtained from velocity error_ Controller_ Vx and 
        #two controllers of the same type corresponding to the y direction)
        move_controller_x = PID_Controller(1.3, 0, 0.4, -0.6, 0.6) 
        move_controller_vx = PID_Controller(30,0, 0.01, -18, 18)
        move_controller_y = PID_Controller(1.3, 0, 0.4, -0.6, 0.6) 
        move_controller_vy = PID_Controller(30,0, 0.01, -18, 18)       
        rate.sleep()
        while not rospy.is_shutdown():
            if state == 0:
                print("x,y",x,y)
                #set the error of x and y
                error_x = 4- x
                error_y = 3-y
                #When reaching the target position, all errors are cleared to zero and transfer State
                if abs(error_x) < move_er_threshold :
                    state = state + 1
                    twist.linear.x = 0
                    twist.linear.y = 0
                    twist.angular.z = 0
                    move_controller_x.error = 0  
                    move_controller_x.last_error = 0
                    move_controller_x.error_sum = 0

                    move_controller_y.error = 0  
                    move_controller_y.last_error = 0
                    move_controller_y.error_sum = 0

                    move_controller_vx.error = 0  
                    move_controller_vx.last_error = 0
                    move_controller_vx.error_sum = 0

                    move_controller_vy.error = 0  
                    move_controller_vy.last_error = 0
                    move_controller_vy.error_sum = 0

                else:
                    #Calculate target speed from position error
                    temp_x  = move_controller_x.get_output(error_x)                 
                    temp_y = move_controller_y.get_output(error_y)
                    #Calculate the speed error
                    error_vx = temp_x-vx
                    error_vy = temp_y - vy
                    #Calculate target force from speed error
                    twist.linear.x = move_controller_vx.get_output(error_vx)
                    twist.linear.y = move_controller_vy.get_output(error_vy)
                    twist.linear.y = min(twist.linear.y, 3.0*twist.linear.x/4)                 
                    twist.angular.z = 0
            elif state == 1:
                print("x,y",x,y)
                error_x = 2- x
                error_y = 5- y
                if abs(error_y) < move_er_threshold:
                    state = state + 1
                    twist.linear.x = 0
                    twist.linear.y = 0
                    twist.angular.z = 0

                    move_controller_x.error = 0  
                    move_controller_x.last_error = 0
                    move_controller_x.error_sum = 0

                    move_controller_y.error = 0  
                    move_controller_y.last_error = 0
                    move_controller_y.error_sum = 0

                    move_controller_vx.error = 0  
                    move_controller_vx.last_error = 0
                    move_controller_vx.error_sum = 0

                    move_controller_vy.error = 0  
                    move_controller_vy.last_error = 0
                    move_controller_vy.error_sum = 0
                else:
                    temp_x  = move_controller_x.get_output(error_x)
                    
                    temp_y = move_controller_y.get_output(error_y)
                
                    # temp_y = temp_y - vy_sum
                    # #if twist.linear.x > 0 , the direction of F is -x
                    twist.linear.x = move_controller_vx.get_output(temp_x- vx)
                    # print("temp_x",temp_x)
                    # #if twist.linear.y > 0 , the direction of F is -y
                    twist.linear.y = move_controller_vy.get_output(temp_y - vy)
                    #twist.linear.y = min(twist.linear.y, abs(1.0*twist.linear.x/6))
            elif state == 2:
                print("x,y",x,y)
                error_x = 0-x
                error_y = 3-y 
                if abs(error_x) < move_er_threshold :
                # if abs(error_x) < move_er_threshold:
                    state = state + 1
                    twist.linear.x = 0
                    twist.linear.y = 0
                    twist.angular.z = 0
                    move_controller_x.error = 0  
                    move_controller_x.last_error = 0
                    move_controller_x.error_sum = 0

                    move_controller_y.error = 0  
                    move_controller_y.last_error = 0
                    move_controller_y.error_sum = 0

                    move_controller_vx.error = 0  
                    move_controller_vx.last_error = 0
                    move_controller_vx.error_sum = 0

                    move_controller_vy.error = 0  
                    move_controller_vy.last_error = 0
                    move_controller_vy.error_sum = 0

                else:
                    temp_x  = move_controller_x.get_output(error_x)                 
                    temp_y = move_controller_y.get_output(error_y)
                    error_vx = temp_x-vx
                    error_vy = temp_y - vy
                    #twist.linear.x = 0
                    twist.linear.x = move_controller_vx.get_output(error_vx)
                    twist.linear.y = move_controller_vy.get_output(error_vy)
                    #twist.linear.x = min(twist.linear.x , 1.0* twist.linear.y/6)
                    twist.angular.z = 0
            elif state == 3:
                print("x,y",x,y)
                error_x = 0- x
                error_y = 0- y
                if abs(error_y) < move_er_threshold:
                    state = (state + 1)%4
                    twist.linear.x = 0
                    twist.linear.y = 0
                    twist.angular.z = 0

                    move_controller_x.error = 0  
                    move_controller_x.last_error = 0
                    move_controller_x.error_sum = 0

                    move_controller_y.error = 0  
                    move_controller_y.last_error = 0
                    move_controller_y.error_sum = 0

                    move_controller_vx.error = 0  
                    move_controller_vx.last_error = 0
                    move_controller_vx.error_sum = 0

                    move_controller_vy.error = 0  
                    move_controller_vy.last_error = 0
                    move_controller_vy.error_sum = 0
                else:
                    temp_x  = move_controller_x.get_output(error_x)                    
                    temp_y = move_controller_y.get_output(error_y)
                   
                    # #if twist.linear.x > 0 , the direction of F is -x
                    twist.linear.x = move_controller_vx.get_output(temp_x- vx)
                    # print("temp_x",temp_x)
                    # #if twist.linear.y > 0 , the direction of F is -y
                    twist.linear.y = move_controller_vy.get_output(temp_y - vy)
                    #twist.linear.y = min(twist.linear.y , 2*twist.linear.x/3)
            self.pub.publish(twist) 
            rospy.loginfo("state: %0.1f",state)
            #rospy.loginfo("Twist: [%0.2f m/s, %0.2f m/s,%0.2f rad/s] state: %0.1f", vx ,vy ,twist.angular.z, state)   
            rate.sleep()
        sys.exit(0)

if __name__ == '__main__':
    try:
        rospy.init_node('controller',anonymous=True)
        vel = controller()
        vel.velocity_publisher()
    except rospy.ROSInterruptException:
        pass