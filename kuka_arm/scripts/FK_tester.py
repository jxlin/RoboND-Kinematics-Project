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
        self.m_subsTf = rospy.Subscriber( '/tf',
                                          TFMessage,
                                          self.onTFMsgCallback )
        # Subscribe to the tf_static topic, for the offset of the gripper
        self.m_subsTfStatic = rospy.Subscriber( '/tf_static',
                                                TFMessage,
                                                self.onTFStaticMsgCallback )

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
        self.m_rvizEEoffset = None
        self.m_rvizTransforms = [ None ] * 7
        self.m_modelTransforms = [ None ] * 7

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

    def compareTransforms( self ):
        # TODO: Implement comparison
        pass

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

        # _poseDH.position.x = self.m_gpos[0,0]
        # _poseDH.position.y = self.m_gpos[1,0]
        # _poseDH.position.z = self.m_gpos[2,0]
        # _poseDH.orientation.x = self.m_gquat[0]
        # _poseDH.orientation.y = self.m_gquat[1]
        # _poseDH.orientation.z = self.m_gquat[2]
        # _poseDH.orientation.w = self.m_gquat[3]

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

        self.m_pubPoseFK.publish( _poseDH )
        self.m_pubPoseRVIZ.publish( _poseRVIZ )

    def run( self ) :

        while not rospy.is_shutdown() :

            self.computeEEpose()
            self.compareTransforms()
            self.publishPoseComparison()

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

    _node = FK_tester()

    try :
        _node.run()
    except rospy.ROSInterruptException :
        pass