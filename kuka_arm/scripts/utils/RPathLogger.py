

### import modules
# helper libraries
import numpy as np
import os

FILEPATH_TRAJECTORY_TO_TARGET  = 'f_path_to_target.txt'
FILEPATH_TRAJECTORY_TO_DROPOFF = 'f_path_to_dropoff.txt'

class RPose :

    def __init__( self, position, orientation ) :

        self.position = position
        self.orientation = orientation

class RPathLogger( object ) :


    def __init__( self ) :

        super( RPathLogger, self ).__init__()

        self.m_trajectoryToTarget = []
        self.m_trajectoryToDropoff = []

        _prevPath = os.getcwd()

        os.chdir( '/home/gregor/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/logs/' )

        self._loadTrajectory( self.m_trajectoryToTarget,
                              FILEPATH_TRAJECTORY_TO_TARGET )

        self._loadTrajectory( self.m_trajectoryToDropoff,
                              FILEPATH_TRAJECTORY_TO_DROPOFF )

        os.chdir( _prevPath )


    def getTrajectoryToDropoff( self ) :
        return self.m_trajectoryToDropoff

    def getTrajectoryToTarget( self ) :
        return self.m_trajectoryToTarget

    def _loadTrajectory( self, trajectoryBuff, filepath ) :

        _poses = np.transpose( np.loadtxt( filepath, delimiter = ';', unpack = True ) )

        for i in range( len( _poses ) ) :
            _position    = _poses[i][0:3]
            _orientation = _poses[i][3:]

            _pose = RPose( _position, _orientation )

            trajectoryBuff.append( _pose )