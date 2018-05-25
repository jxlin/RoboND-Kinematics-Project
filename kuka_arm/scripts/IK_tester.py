#!/usr/bin/env python

### import modules
# Ros modules
import rospy
import tf
# Service request definitions
from kuka_arm.srv import *
# Messages definitions
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from tf2_msgs.msg import TFMessage
# helper libraries
from mpmath import * # for arbitraty floating point precision operations
from sympy import * # for symbolic operations

# R-DH library
from dhlibrary.RDHmodelKukaKR210 import *




class FK_tester :


    def __init__( self ) :
        # Initialize ros node
        rospy.init_node( 'FK_tester' )

        # Create the DH model of the kuka kr210 arm
        self.m_model = RDHmodelKukaKR210()

        # Subscribe to the joint publisher's topic
        self.m_subsJoints = rospy.Subscriber( '/joint_states',
                                              JointState,
                                              self.onJointMsgCallback )
        # Subscribe to the tf topic, to compare results
        self.m_subsPose = rospy.Subscriber( '/tf',
                                            TFMessage,
                                            self.onTFMsgCallback )

        # Publish the final pose and compare it with a plot
        self.m_pubPoseFK    = rospy.Publisher( '/EE_pose_FK', Pose, queue_size = 10 )
        self.m_pubPoseRVIZ  = rospy.Publisher( '/EE_pose_RVIZ', Pose, queue_size = 10 )

        # joint angles
        self.m_jointAngles = [ 0 ] * 6
        # position and orientation from last TF ( base_link to gripper_link )
        self.m_gpos = np.zeros( ( 3, 1 ) )
        self.m_grpy = np.zeros( ( 3, 1 ) )
        self.m_gquat = None
        # transform for each link
        self.m_rvizTransforms = [ None ] * 6
        self.m_modelTransforms = [ None ] * 6

        # Create some other helpful objects
        self.m_rate = rospy.Rate( 10 )

    def computeEEpose( self ) :
        # Apply forward kinematics
        self.m_model.forward( self.m_jointAngles )
        # update the model
        self.m_model.updateModel()
        # compute gripper pos and rot
        self.m_gpos = self.m_model.getEndEffectorXYZ()
        self.m_grpy = self.m_model.getEndEffectorRPY()
        # self.m_gquat = tf.transformations.quaternion_from_euler( self.m_grpy[0,0],
        #                                                          self.m_grpy[1,0],
        #                                                          self.m_grpy[2,0] )
        self.m_gquat = tf.transformations.quaternion_from_matrix( self.m_model.getTotalEndEffectorTransform() )

        # print 'compensation:'
        # print self.m_model.getEndEffectorCompensation()

        # print 'lastframe(xyz):'
        # print self.m_model.getLastFrameXYZ()

        # print 'endEffector(xyz):'
        # print self.m_model.getEndEffectorXYZ()

        print 'EEpos:'
        print self.m_gpos

        print 'EErpy:'
        print self.m_grpy

        print 'EEquat:'
        print self.m_gquat

    def publishPose( self ) :
        # TODO: Implement this for report
        pass

    def run( self ) :

        while not rospy.is_shutdown() :

            self.computeEEpose()
            self.publishPose()

            self.m_rate.sleep()

    def onJointMsgCallback( self, jointMsg ) :
        # Collect joint angles
        self.m_jointAngles = [ jointMsg.position[i] for i in range( 2, len( jointMsg.position ) ) ]

    def onTFMsgCallback( self, tfMsg ) :
        # Collect transforms for each link

        # _transforms = tfMsg.transforms
        # self.m_rvizTransforms = 

        pass

if __name__ == '__main__' :

    _node = FK_tester()

    try :
        _node.run()
    except rospy.ROSInterruptException :
        pass