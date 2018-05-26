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




class IK_tester :


    def __init__( self ) :
        # Initialize ros node
        rospy.init_node( 'IK_tester' )

        # Create the DH model of the kuka kr210 arm
        self.m_model = RDHmodelKukaKR210()

        # Subscribe to the IK_pose_reference topic to get a pose each time

        # Subscribe to the IK_trajectory_reference topic to get a full trajectory

        # Subscribe to the tf topic, to compare results
        self.m_subsTf = rospy.Subscriber( '/tf',
                                          TFMessage,
                                          self.onTFMsgCallback )
        # Subscribe to the tf_static topic, for the offset of the gripper
        self.m_subsTfStatic = rospy.Subscriber( '/tf_static',
                                                TFMessage,
                                                self.onTFStaticMsgCallback )

        # Publish the final pose and compare it with a plot
        self.m_pubPoseIK      = rospy.Publisher( '/EE_pose_IK', Pose, queue_size = 10 )
        self.m_pubPoseRequest = rospy.Publisher( '/EE_pose_Request', Pose, queue_size = 10 )

        # joint angles
        self.m_jointAngles = [ 0 ] * 6
        # position and orientation from last TF ( base_link to gripper_link )
        self.m_refPos  = np.zeros( ( 3, 1 ) )
        self.m_refRpy  = np.zeros( ( 3, 1 ) )
        self.m_refQuat = np.zeros( ( 4, 1 ) )
        # transform for each link
        self.m_rvizEEoffset = None
        self.m_rvizTransforms = [ None ] * 7
        self.m_modelTransforms = [ None ] * 7

        # Create some other helpful objects
        self.m_rate = rospy.Rate( 10 )

    def computeJointsFromEEpose( self ) :
        # Apply forward kinematics
        self.m_model.inverse( self.m_refPos, self.m_refRpy )
        # update the model
        self.m_model.updateModel()

    def publishPoseComparison( self ) :
        
        if self.m_rvizTransforms[-1] is None :
            # Go back till we have all the rviz transforms that we need
            return

        # Build pose of End Effector from our dh-model
        _positionDH = tf.transformations.translation_from_matrix( self.m_modelTransforms[-1] )
        _orientationDH = tf.transformations.quaternion_from_matrix( self.m_modelTransforms[-1] )

        _poseDH = Pose()
        _poseDH.position.x = _positionDH[0]
        _poseDH.position.y = _positionDH[1]
        _poseDH.position.z = _positionDH[2]
        _poseDH.orientation.x = _orientationDH[0]
        _poseDH.orientation.y = _orientationDH[1]
        _poseDH.orientation.z = _orientationDH[2]
        _poseDH.orientation.w = _orientationDH[3]

        # _poseDH.position.x = self.m_refPos[0,0]
        # _poseDH.position.y = self.m_refPos[1,0]
        # _poseDH.position.z = self.m_refPos[2,0]
        # _poseDH.orientation.x = self.m_refQuat[0]
        # _poseDH.orientation.y = self.m_refQuat[1]
        # _poseDH.orientation.z = self.m_refQuat[2]
        # _poseDH.orientation.w = self.m_refQuat[3]

        # Get pose of End Effector from rviz-transforms
        _positionRVIZ = tf.transformations.translation_from_matrix( self.m_rvizTransforms[-1] )
        _orientationRVIZ = tf.transformations.quaternion_from_matrix( self.m_rvizTransforms[-1] )

        _poseRVIZ = Pose()
        _poseRVIZ.position.x = _positionRVIZ[0]
        _poseRVIZ.position.y = _positionRVIZ[1]
        _poseRVIZ.position.z = _positionRVIZ[2]
        _poseRVIZ.orientation.x = _orientationRVIZ[0]
        _poseRVIZ.orientation.y = _orientationRVIZ[1]
        _poseRVIZ.orientation.z = _orientationRVIZ[2]
        _poseRVIZ.orientation.w = _orientationRVIZ[3]

        self.m_pubPoseIK.publish( _poseDH )
        self.m_pubPoseRequest.publish( _poseRVIZ )

    def run( self ) :

        while not rospy.is_shutdown() :

            self.computeJointsFromEEpose()
            self.compareTransforms()
            self.publishPose()

            self.m_rate.sleep()

    def onJointMsgCallback( self, jointMsg ) :
        # Collect joint angles
        self.m_jointAngles = [ jointMsg.position[i] for i in range( 2, len( jointMsg.position ) ) ]

    def onTFMsgCallback( self, tfMsg ) :
        # Collect transforms for each link ( non EE transform )
        _transforms = tfMsg.transforms
        _tf = np.eye( 4 )

        # Get only the link-link transforms, not the fingers-link
        for i in range( 6 ) :
            _translation = _transforms[i].transform.translation
            _rotation    = _transforms[i].transform.rotation

            _tfTrans = tf.transformations.translation_matrix( vec3d2numpy( _translation ) )
            _tfRot   = tf.transformations.quaternion_matrix( quat2numpy( _rotation ) )
            _tf = np.dot( _tf, np.dot( _tfTrans, _tfRot ) )

            self.m_rvizTransforms[i] = _tf
            self.m_modelTransforms[i] = self.m_model.getTransformInRange( 0, i )

        if self.m_rvizEEoffset is not None :
            self.m_rvizTransforms[6] = np.dot( self.m_rvizTransforms[5], self.m_rvizEEoffset )
            self.m_modelTransforms[6] = self.m_model.getTotalEndEffectorTransform()

    def onTFStaticMsgCallback( self, tfMsg ):
        # Extract transform offset for the gripper link ( link6 to gripper_link )
        _transforms = tfMsg.transforms
        _lastFrameToGripper = _transforms[1].transform

        _tfTrans = tf.transformations.translation_matrix( vec3d2numpy( _lastFrameToGripper.translation ) )
        _tfRot   = tf.transformations.quaternion_matrix( quat2numpy( _lastFrameToGripper.rotation ) )
        
        # Compute gripper link offset transform
        self.m_rvizEEoffset = np.dot( _tfTrans, _tfRot )

if __name__ == '__main__' :

    _node = IK_tester()

    try :
        _node.run()
    except rospy.ROSInterruptException :
        pass