#!/usr/bin/env python

### import modules
# Ros modules
import rospy
import tf
# Service request definitions
from kuka_arm.srv import *
# Messages definitions
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseArray
from tf2_msgs.msg import TFMessage
# helper libraries
from mpmath import * # for arbitraty floating point precision operations
from sympy import * # for symbolic operations

# R-DH library
from dhlibrary.RDHmodelKukaKR210 import *

IK_MODE_IDLE        = 0
IK_MODE_SINGLE_POSE = 1
IK_MODE_TRAJECTORY  = 2

# Just in cases where some stuff went wrong, and ...
# need to check every dh - computation ( inverse, most of the time )
LOG_EVERYTHING_FUUUU = False

class TrajectoryPose:

    def __init__( self ) :
        self.position = np.zeros( ( 3, 1 ) )
        self.orientation = np.zeros( ( 3, 1 ) )

class IK_tester :


    def __init__( self ) :
        # Initialize ros node
        rospy.init_node( 'IK_tester' )

        # Create the DH model of the kuka kr210 arm
        self.m_model = RDHmodelKukaKR210()

        # Subscribe to the IK_pose_reference topic to get a pose each time
        self.m_subsIKreferenceSinglePose = rospy.Subscriber( '/IK_pose_reference',
                                                             Pose,
                                                             self.onSinglePoseMsgCallback )
        # Subscribe to the IK_trajectory_reference topic to get a full trajectory
        self.m_subsIKreferenceTrajectory = rospy.Subscriber( '/IK_trajectory_reference',
                                                             PoseArray,
                                                             self.onTrajectoryMsgCallback )
        # Subscribe to the tf topic, to compare results
        self.m_subsTf = rospy.Subscriber( '/tf',
                                          TFMessage,
                                          self.onTFMsgCallback )
        # Subscribe to the tf_static topic, for the offset of the gripper
        self.m_subsTfStatic = rospy.Subscriber( '/tf_static',
                                                TFMessage,
                                                self.onTFStaticMsgCallback )

        # Publish the final pose and compare it with a plot
        self.m_pubPoseRVIZ    = rospy.Publisher( '/EE_pose_RVIZ', Pose, queue_size = 10 )
        self.m_pubPoseRequest = rospy.Publisher( '/EE_pose_Request', Pose, queue_size = 10 )
        # Publish the joint angles for robot_state_publisher to use them
        self.m_pubJoints = rospy.Publisher( '/joint_states', JointState, queue_size = 10 )

        # joint angles
        self.m_jointAngles = [ 0 ] * 6
        # position and orientation from last TF ( base_link to gripper_link )
        self.m_refPos  = np.zeros( ( 3, 1 ) )
        self.m_refRpy  = np.zeros( ( 3, 1 ) )
        self.m_refQuat = np.zeros( ( 4, 1 ) )
        # transform for each link - just look at the rviz transforms, not our ...
        # model transforms for comparison
        self.m_rvizEEoffset = None
        self.m_rvizTransforms = [ None ] * 7

        # To check if should hold until a trajectory has finished
        self.m_ikExecutionMode = IK_MODE_IDLE

        # Trajectory execution info
        self.m_tBef = rospy.get_time()
        self.m_tNow = rospy.get_time()
        self.m_tDelta = 0.0
        self.m_timer = 0.0
        self.m_trajectory = None
        self.m_currentTrajectoryPoseIndx = -1

        # Create some other helpful objects
        self.m_rate = rospy.Rate( 10 )

    def onSinglePoseMsgCallback( self, poseMsg ) :

        if self.m_ikExecutionMode == IK_MODE_IDLE :

            self.m_refPos[0,0] = poseMsg.position.x
            self.m_refPos[1,0] = poseMsg.position.y
            self.m_refPos[2,0] = poseMsg.position.z

            self.m_refQuat[0,0] = poseMsg.orientation.x
            self.m_refQuat[1,0] = poseMsg.orientation.y
            self.m_refQuat[2,0] = poseMsg.orientation.z
            self.m_refQuat[3,0] = poseMsg.orientation.w

            _rpy = tf.transformations.euler_from_quaternion( [ self.m_refQuat[0,0], self.m_refQuat[1,0],
                                                               self.m_refQuat[2,0], self.m_refQuat[3,0] ] )
            self.m_refRpy[0,0] = _rpy[0]
            self.m_refRpy[1,0] = _rpy[1]
            self.m_refRpy[2,0] = _rpy[2]

            self.m_ikExecutionMode = IK_MODE_SINGLE_POSE

    def onTrajectoryMsgCallback( self, poseArrayMsg ) :
        
        if self.m_ikExecutionMode == IK_MODE_IDLE :

            self.m_timer = 0.0
            self.m_trajectory = []
            self.m_currentTrajectoryPoseIndx = -1

            for i in range( len( poseArrayMsg.poses ) ) :
                _pose = poseArrayMsg.poses[i]

                _tPose = TrajectoryPose()
                
                _tPose.position[0,0] = _pose.position.x
                _tPose.position[1,0] = _pose.position.y
                _tPose.position[2,0] = _pose.position.z

                _rpy = tf.transformations.euler_from_quaternion( [ _pose.orientation.x, _pose.orientation.y,
                                                                   _pose.orientation.z, _pose.orientation.w ] )

                _tPose.orientation[0,0] = _rpy[0]
                _tPose.orientation[1,0] = _rpy[1]
                _tPose.orientation[2,0] = _rpy[2]

                self.m_trajectory.append( _tPose )

            self.m_ikExecutionMode = IK_MODE_TRAJECTORY

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

        if self.m_rvizEEoffset is not None :
            self.m_rvizTransforms[6] = np.dot( self.m_rvizTransforms[5], self.m_rvizEEoffset )

    def onTFStaticMsgCallback( self, tfMsg ):
        # Extract transform offset for the gripper link ( link6 to gripper_link )
        _transforms = tfMsg.transforms
        _lastFrameToGripper = _transforms[1].transform

        _tfTrans = tf.transformations.translation_matrix( vec3d2numpy( _lastFrameToGripper.translation ) )
        _tfRot   = tf.transformations.quaternion_matrix( quat2numpy( _lastFrameToGripper.rotation ) )
        
        # Compute gripper link offset transform
        self.m_rvizEEoffset = np.dot( _tfTrans, _tfRot )

    def run( self ) :

        while not rospy.is_shutdown() :

            self.m_tNow = rospy.get_time()
            self.m_tDelta = self.m_tNow - self.m_tBef
            self.m_tBef = self.m_tNow

            self.m_timer += self.m_tDelta

            # Check mode and act accordingly
            if self.m_ikExecutionMode == IK_MODE_SINGLE_POSE :
                self._executionModeSingle()
            elif self.m_ikExecutionMode == IK_MODE_TRAJECTORY :
                self._executionModeTrajectory()

            # Publish joints for rviz usage
            self.publishJoints()
            # Publish pose comparisons for plotting
            self.publishPoseComparison()

            self.m_rate.sleep()

    def _executionModeSingle( self ) :
        # Apply inverse kinematics
        self.m_jointAngles = self.m_model.inverse( self.m_refPos, self.m_refRpy )

        # Check if the position was solvable
        if self.m_jointAngles is not None :
            # update the model
            self.m_model.updateModel()
        # else :
        #     # Log the error
        #     self.m_model._logToFile( [], self.m_refPos, self.m_refRpy, 'IK error' )

        # set mode back to idle
        self.m_ikExecutionMode = IK_MODE_IDLE

    def _executionModeTrajectory( self ) :
        _poseRef = self._getPose()
        
        if _poseRef is None :
            # Have finished trajectory
            self.m_ikExecutionMode = IK_MODE_IDLE
            return

        self.m_refPos = _poseRef.position
        self.m_refRpy = _poseRef.orientation

        _quat = tf.transformations.quaternion_from_euler( self.m_refRpy[0,0],
                                                          self.m_refRpy[1,0],
                                                          self.m_refRpy[2,0] )
        self.m_refQuat[0,0] = _quat[0]
        self.m_refQuat[1,0] = _quat[1]
        self.m_refQuat[2,0] = _quat[2]
        self.m_refQuat[3,0] = _quat[3]

        self.m_jointAngles = self.m_model.inverse( self.m_refPos, self.m_refRpy )

        # Check if the position was solvable
        if self.m_jointAngles is not None :
            # update the model
            self.m_model.updateModel()
            # Log all the path computations, just in case
            if LOG_EVERYTHING_FUUUU :
                self.m_model._logToFile( self.m_jointAngles,
                                         self.m_refPos, self.m_refRpy,
                                         'IK Ok!!!' )
        else :
            # Log the error
            self.m_model._logToFile( [], self.m_refPos, self.m_refRpy, 'IK error' )

    def _getPose( self ) :

        if self.m_currentTrajectoryPoseIndx == -1 :
            self.m_currentTrajectoryPoseIndx = 0
            print 'PoseIndx: ', 0

        elif self.m_timer > 0.2 :
            self.m_timer = 0
            if self.m_currentTrajectoryPoseIndx >= ( len( self.m_trajectory ) - 1 ) :
                self.m_currentTrajectoryPoseIndx = -1
                print 'DONE WITH TRAJECTORY!'
                return None
            else :
                self.m_currentTrajectoryPoseIndx += 1
                print 'PoseIndx: ', self.m_currentTrajectoryPoseIndx

        return self.m_trajectory[ self.m_currentTrajectoryPoseIndx ]
        
    def publishJoints( self ) :

        # Only send if previous position was solvable
        if self.m_jointAngles is not None :

            # make JointState msg
            _msg = JointState()
            _msg.header.stamp = rospy.Time.now()
            _msg.name = [ 'right_gripper_finger_joint',
                          'left_gripper_finger_joint',
                          'joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6' ]
            _msg.position = [ 0.0, 0.0,
                              self.m_jointAngles[0],
                              self.m_jointAngles[1],
                              self.m_jointAngles[2],
                              self.m_jointAngles[3],
                              self.m_jointAngles[4],
                              self.m_jointAngles[5] ]

            # publish this message
            self.m_pubJoints.publish( _msg )

            # print 'joints OK!> ', self.m_jointAngles
            print 'joints OK!'

    def publishPoseComparison( self ) :
        
        if self.m_rvizTransforms[-1] is None :
            # Go back till we have all the rviz transforms that we need
            return

        # Build pose of End Effector from requested reference
        _poseRequest = Pose()
        _poseRequest.position.x = self.m_refPos[0,0]
        _poseRequest.position.y = self.m_refPos[1,0]
        _poseRequest.position.z = self.m_refPos[2,0]

        _poseRequest.orientation.x = self.m_refQuat[0,0]
        _poseRequest.orientation.y = self.m_refQuat[1,0]
        _poseRequest.orientation.z = self.m_refQuat[2,0]
        _poseRequest.orientation.w = self.m_refQuat[3,0]

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

        self.m_pubPoseRVIZ.publish( _poseRVIZ )
        self.m_pubPoseRequest.publish( _poseRequest )

if __name__ == '__main__' :

    _node = IK_tester()

    try :
        _node.run()
    except rospy.ROSInterruptException :
        pass