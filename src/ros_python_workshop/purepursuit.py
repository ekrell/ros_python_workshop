#!/usr/bin/env python

import rospy
from turtlesim.msg import *
from turtlesim.srv import *
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from std_srvs.srv import *
import time, sys, math

pose = { "x"     : None,
         "y"     : None,
         "theta" : None,
       } 


class turtlebot_velocity (object): 
    _pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size = 10)

    def __init__(self):
        rospy.init_node('turtlebot_trajectoryFollow')
        self.pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size = 10)
        rospy.sleep(1)

    def write(self, linear, angular):
        self._pub.publish(Twist(Vector3(linear, 0, 0), Vector3(0, 0, angular)))

def turtlebot_odometry_callback(msg):
    global pose
    pose["x"]     = msg.x
    pose["y"]     = msg.y
    pose["theta"] = msg.theta

def listener():
    return rospy.Subscriber("/turtle1/pose", Pose, turtlebot_odometry_callback)

def setParams(params):
    if rospy.has_param("min_dist_goal"):
        params["min_dist_goal"] = rospy.get_param("min_dist_goal")
    else:
        rospy.set_param("min_dist_goal", params["min_dist_goal"])
    if rospy.has_param("dist_wheel_center"):
        params["dist_wheel_center"] = rospy.get_param("dist_wheel_center")
    else:
        rospy.set_param("dist_wheel_center", params["dist_wheel_center"])
    if rospy.has_param("k"):
        params["k"] = rospy.get_param("k")
    else:
        rospy.set_param("k", params["k"])
    if rospy.has_param("look_ahead"):
        params["look_ahead"] = rospy.get_param("look_ahead")
    else:
        rospy.set_param("look_ahead", params["look_ahead"])
    if rospy.has_param("kp"):
        params["kp"] = rospy.get_param("kp")
    else:
        rospy.set_param("kp", params["kp"])
    if rospy.has_param("l"):
        params["l"] = rospy.get_param("l")
    else:
        rospy.set_param("l", params["l"])
    if rospy.has_param("linear_speed"):
        params["linear_speed"] = rospy.get_param("linear_speed")
    else:
        rospy.set_param("linear_speed", params["linear_speed"])
    if rospy.has_param("goal"):
        params["goal"] = rospy.get_param("goal")
    else:
        rospy.set_param("goal", params["goal"])

def updateParams(params):
    if rospy.has_param("min_dist_goal"):
        params["min_dist_goal"] = rospy.get_param("min_dist_goal")
    if rospy.has_param("dist_wheel_center"):
        params["dist_wheel_center"] = rospy.get_param("dist_wheel_center")
    if rospy.has_param("k"):
        params["k"] = rospy.get_param("k")
    if rospy.has_param("look_ahead"):
        params["look_ahead"] = rospy.get_param("look_ahead")
    if rospy.has_param("kp"):
        params["kp"] = rospy.get_param("kp")
    if rospy.has_param("l"):
        params["l"] = rospy.get_param("l")
    if rospy.has_param("linear_speed"):
        params["linear_speed"] = rospy.get_param("linear_speed")
    if rospy.has_param("goal"):
        params["goal"] = rospy.get_param("goal")

def status2str(pose, goal):
    return "Position: (x:{}, y:{}, theta:{}), Goal: (x:{}, y:{})".format(pose["x"], 
                                        pose["y"], pose["theta"], goal[0], goal[1])
   
def main():

    ##############
    # Parameters #
    ##############
    params = { "min_dist_goal"     : 0.5,
               "dist_wheel_center" : 0.005,
               "k"                 : 0.1,
               "look_ahead"        : 1,
               "kp"                : 1.0,
               "l"                 : 2.9,
               "linear_speed"      : 1,
               "goal"              : (9, 9),
             }
    setParams(params)

    #########
    # Setup #
    #########

    # Initialize ROS connections
    rospy.loginfo("Setting up ROS connections..")
    velctrl = turtlebot_velocity()
    sub = listener()
    rospy.loginfo("ROS connections established")
    
    # Start timer (for logging, etc)
    startTime = time.time()
    iteration = 0
    # Give ROS a chance to start up
    rospy.sleep (10)  # Typical value is 1, but turtlesim_node is slow
    # Init inf distance to goal 
    dist2Goal = float("inf")

    rospy.loginfo_throttle(10, status2str(pose, params["goal"]))

    # Start PurePursuit loop
    while (True):
        # Slight delay
        rospy.sleep(0.05)
        # Get updated parameters from parameter server
        if iteration % 100 == 0:
            updateParams(params)

        # Update iteration
        iteration = iteration + 1
        endTime = time.time()
        runTime = endTime - startTime

        #########
        # SENSE #
        #########

        # Get robot current position
        if (pose == None):
            continue

        ###########
        # Process #
        ###########

        # Distance to goal
        dist2goal = pow(((pose["x"]-params["goal"][0])*(pose["x"]-params["goal"][0]) + \
                         (pose["y"]-params["goal"][1])*(pose["y"]-params["goal"][1])), 0.5)
        if (dist2goal < params["min_dist_goal"]):
            continue

        # Calculate curvature
        dx = pose['x'] - params["goal"][0]
        dy = pose['y'] - params["goal"][1]
        x1 = math.cos(pose["theta"]) * dx + math.sin(pose["theta"]) * dy 
        y1 = -1 * math.sin(pose["theta"]) * dx + math.cos(pose["theta"]) * dy
        if (x1 * x1 + y1 * y1) == 0:
            curv = 0
        else:
            curv = (1.0 / (x1 * x1 + y1 * y1)) * (-1 * y1)

        ###########
        # CONTROL #
        ###########

        # Send movement control to robot
        velctrl.write(params["linear_speed"], curv * params["linear_speed"])

        #############
        # Visualize #
        #############
        
        # handled by the invocation of rospy.loginfo_throttle
        # view messages with rostopic echo /rosout


if __name__ == '__main__':
    main()

