#! /usr/bin/env python


## @package rt2_assignment1
#
# \file go_to_point.py
# \brief  This file mainly implement the math necessary to move the robot
# \author Roberta Alessi
# \version 0.1
#
# \details
#
#
# Subscribes to: <BR>
#    /odom topic
#
# Publishes to: <BR>
#    /cmd_vel
# 
# Action Clients: <BR>
#    /reaching_goal
#
#

import rospy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
from rt2_assignment1.srv import Position
import math
import actionlib
import actionlib.msg
import rt2_assignment1.msg

# robot state variables
position_ = Point()
yaw_ = 0
position_ = 0
state_ = 0
pub_ = None

#goal definition
desired_position = Point()

# parameters for control
yaw_precision_ = math.pi / 9  # +/- 20 degree allowed
yaw_precision_2_ = math.pi / 90  # +/- 2 degree allowed
dist_precision_ = 0.1
kp_a = -3.0 
kp_d = 0.2
ub_a = 0.6
lb_a = -0.5
ub_d = 0.6

##
#
# \brief callback of odometry subscriber.
# \param position_
# \param yaw_
# 
#
#  Description :
#  It reads the msg containing the information about the robot position and yaw.
#  At the end, it computes the euler angle for the yaw from the given quaternion
#
#

def clbk_odom(msg):
    global position_
    global yaw_

    # position
    position_ = msg.pose.pose.position

    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]


def change_state(state):
    global state_
    state_ = state
    print ('State changed to [%s]' % state_)

##
#
# \brief function that normalize the angle
#
#

def normalize_angle(angle):
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle

##
#
# \brief This function adjusts the angle by taking in input the goal of the robot.
#
#
#  Description :
#  Once the robot is rotated of the right angle (taking in cosideration the angle of the goal), it moves toward that direction 
#  with a certain velocity published by the twist message.
#

def fix_yaw(des_pos):
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = normalize_angle(desired_yaw - yaw_)
    #rospy.loginfo(err_yaw)
    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_2_:
        twist_msg.angular.z = kp_a*err_yaw
        if twist_msg.angular.z > ub_a:
            twist_msg.angular.z = ub_a
        elif twist_msg.angular.z < lb_a:
            twist_msg.angular.z = lb_a
    pub_.publish(twist_msg)
    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_2_:
        #print ('Yaw error: [%s]' % err_yaw)
        change_state(1)

##
#
# \brief This function adjust the velocities to reach the goal. 
#
#
# Description
# If the error from computing the distance position, between the goal and the actual postion, is greater than a certain threshold
# then the robot can slow down and adjust its orientantion and move again directly to the goal
# 
#

def go_straight_ahead(des_pos):
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = desired_yaw - yaw_
    err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) +
                        pow(des_pos.x - position_.x, 2))
    err_yaw = normalize_angle(desired_yaw - yaw_)
    rospy.loginfo(err_yaw)

    if err_pos > dist_precision_:
        twist_msg = Twist()
        twist_msg.linear.x = 0.3
        if twist_msg.linear.x > ub_d:
            twist_msg.linear.x = ub_d

        twist_msg.angular.z = kp_a*err_yaw
        pub_.publish(twist_msg)
    else: 
        change_state(2)

    # state change conditions
    if math.fabs(err_yaw) > yaw_precision_:
        #print ('Yaw error: [%s]' % err_yaw)
        change_state(0)
##
#
# \brief function that perfoms final yaw adjustments.
#
#

def fix_final_yaw(des_yaw):
    err_yaw = normalize_angle(des_yaw - yaw_)
    rospy.loginfo(err_yaw)
    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_2_:
        twist_msg.angular.z = kp_a*err_yaw
        if twist_msg.angular.z > ub_a:
            twist_msg.angular.z = ub_a
        elif twist_msg.angular.z < lb_a:
            twist_msg.angular.z = lb_a
    pub_.publish(twist_msg)
    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_2_:
        #print ('Yaw error: [%s]' % err_yaw)
        change_state(3)
##
#
# \brief function to stop the robot by setting the linear and angular velocity to 0.
#        
#
def done():
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub_.publish(twist_msg)
    
##
#
# \brief callback function of the action server
# \param desired_position_x
# \param desired_position_y
# \param desired_yaw
# 
#
# Description:
# It takes the goal coordinates and once the robot has reached it (or not) it provides the result and the feedback.
# In function of the robot's position respect to the goal, it can provides feedback about the state, and change it accordingly.
# Moreover, if the goal is preempted it stops the robot.   

def planning(goal):
    global desired_position, act_server, state_

    desired_position.x = goal.x
    desired_position.y = goal.y
    des_yaw = goal.theta
	
    state_ = 0
    rate = rospy.Rate(20)
    success = True
    
    feedback = rt2_assignment1.msg.planningFeedback()
    result = rt2_assignment1.msg.planningResult()
    	
    while not rospy.is_shutdown():
        if  act_server.is_preempt_requested():
             rospy.loginfo('Goal was preempted')
             act_server.set_preempted()
             success = False
             break
        elif state_ == 0:
             feedback.status = "Fixing the yaw"
             act_server.publish_feedback(feedback)
             fix_yaw(desired_position)
        elif state_ == 1:
             feedback.status = "Angle aligned"
             act_server.publish_feedback(feedback)
             go_straight_ahead(desired_position)
        elif state_ == 2:
             feedback.status = "Target reached!"
             act_server.publish_feedback(feedback)
             done()
             break
        else:
            rospy.logerr('Unknown state!')
              
        rate.sleep()
           
    if success:
       rospy.loginfo('Goal: Succeeded!')
       act_server.set_succeeded(result)
##
#
# \brief Main Function
# 
# Here the node is defined as "go_to_point"
#    

def main():
    global pub_, act_server
    
    rospy.init_node('go_to_point')
    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    act_server = actionlib.SimpleActionServer(
        '/reaching_goal', rt2_assignment1.msg.planningAction, planning, auto_start=False)
    act_server.start()

    rate = rospy.Rate(20)
    rospy.spin()
    
    while not rospy.is_shutdown():
         rate.sleep()


if __name__ == '__main__':
    main()
