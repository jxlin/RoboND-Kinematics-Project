#!/usr/bin/env python

# Copyright (C) 2017 Udacity Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

### import modules
# Ros modules
import rospy
import tf
# Service request definitions
from kuka_arm.srv import *
# Messages definitions
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
# helper libraries
from mpmath import * # for arbitraty floating point precision operations
from sympy import * # for symbolic operations

# R-DH library
from dhlibrary.RDHmodelKukaKR210 import *
# DH model for kuka KR210
g_kuka_dh_model = None

def handle_calculate_IK(req):
    global g_kuka_dh_model

    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

        ### Your FK code here
        ## CREATION OF THE DH MODEL WAS MADE BELOW ( in main ) ...
        ## AS A GLOBAL VARIABLE ( g_kuka_dh_model )

        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

            # Extract end-effector position and orientation from request
            # px,py,pz = end-effector position
            # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])

            ### Your IK code here
            ## Extract data for ik solver
            _xyz = np.zeros( ( 3, 1 ) )
            _xyz[0,0] = px
            _xyz[1,0] = py
            _xyz[2,0] = pz

            _rpy = np.zeros( ( 3, 1 ) )
            _rpy[0,0] = roll
            _rpy[1,0] = pitch
            _rpy[2,0] = yaw

            ## Request ik solution
            _joints = g_kuka_dh_model.inverse( _xyz, _rpy )

            ## Assemble solution with a just-in-case check
            theta1 = 0.0
            theta2 = 0.0
            theta3 = 0.0
            theta4 = 0.0
            theta5 = 0.0
            theta6 = 0.0

            if _joints is not None :
                theta1 = _joints[0]
                theta2 = _joints[1]
                theta3 = _joints[2]
                theta4 = _joints[3]
                theta5 = _joints[4]
                theta6 = _joints[5]
            else :
                rospy.logerror( 'Non solvable ik request' )
                print 'Non solvable ik request'

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
            joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
            joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    global g_kuka_dh_model
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    # Create DH-model
    g_kuka_dh_model = RDHmodelKukaKR210()
    # Initialize service
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    # Serve
    rospy.spin()

if __name__ == "__main__":
    IK_server()
