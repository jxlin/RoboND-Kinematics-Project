

# helper libraries
from mpmath import * # for arbitraty floating point precision operations
from sympy import * # for symbolic operations
import numpy as np

# Helper functionality to represent each DH joint
from RDHentry import *
from RDHmath import *



class RDHmodel( object ) :

    def __init__( self ) :

        super( RDHmodel, self ).__init__()

        self.m_dhentries = []

        self.m_totalTransform = np.eye( 4 )
        self.m_xyz = np.zeros( ( 3, 1 ) )
        self.m_rpy = np.zeros( ( 3, 1 ) )

        self.m_endEffectorCompensation = np.eye( 4 )

        self.m_endEffectorTotalTransform = np.eye( 4 )
        self.m_endEffectorXYZ = np.zeros( ( 3, 1 ) )
        self.m_endEffectorRPY = np.zeros( ( 3, 1 ) )

        self._buildModel()

    def getJointTransform( self, indx ) :
        if self._isValidIndex( indx ) :
            return self.m_dhentries[ indx ].getTransform()

        return np.eye( 4 )

    def getTransformInRange( self, fromIndx, toIndx ) :
        if ( 0 <= fromIndx ) and ( fromIndx <= toIndx ) and ( toIndx <= ( self.getNumJoints() - 1 ) ) :
            _cumTransform = np.eye( 4 )

            for i in range( fromIndx, toIndx + 1 ) :
                _transform = self.m_dhentries[ i ].getTransform()
                _cumTransform = np.dot( _cumTransform, _transform )

            return _cumTransform

        return np.eye( 4 )

    def getTotalTransform( self ) :
        return self.m_totalTransform

    def getTotalEndEffectorTransform( self ) :
        return self.m_endEffectorTotalTransform

    def getLastFrameXYZ( self ) :
        return self.m_xyz

    def getLastFrameRPY( self ) :
        return self.m_rpy

    def getEndEffectorXYZ( self ) :
        return self.m_endEffectorXYZ

    def getEndEffectorRPY( self ) :
        return self.m_endEffectorRPY

    def getEndEffectorCompensation( self ) :
        return self.m_endEffectorCompensation

    def updateModel( self ) :
        self.m_totalTransform = self.getTransformInRange( 0, self.getNumJoints() - 1 )
        self.m_xyz[0,0] = self.m_totalTransform[0,3]
        self.m_xyz[1,0] = self.m_totalTransform[1,3]
        self.m_xyz[2,0] = self.m_totalTransform[2,3]
        self.m_rpy = rotationMatrix2euler( self.m_totalTransform )

        self.m_endEffectorTotalTransform = np.dot( self.m_totalTransform, self.m_endEffectorCompensation )
        self.m_endEffectorXYZ[0,0] = self.m_endEffectorTotalTransform[0,3]
        self.m_endEffectorXYZ[1,0] = self.m_endEffectorTotalTransform[1,3]
        self.m_endEffectorXYZ[2,0] = self.m_endEffectorTotalTransform[2,3]
        self.m_endEffectorRPY = rotationMatrix2euler( self.m_endEffectorTotalTransform )

    def forward( self, jointValues ) :

        if len( jointValues ) == self.getNumJoints() :
            for i in range( self.getNumJoints() ) :
                self.setJointValue( i, jointValues[i] )

        else :
            print 'RDHmodel> wrong number of joints sent to forward kinematics: ', len( jointValues )

        self.updateModel()

        return self.m_endEffectorXYZ.copy(), self.m_endEffectorRPY.copy()

    def inverse( self, xyz, rpy, applyToModel = True ) :
        # Override this method in parent
        return []

    def getNumJoints( self ) :
        return len( self.m_dhentries )

    def setJointValue( self, indx, value ) :
        if self._isValidIndex( indx ) :
            self.m_dhentries[ indx ].setJointValue( value )

    def getJointValue( self, indx ) :
        if self._isValidIndex( indx ) :
            return self.m_dhentries[ indx ].getJointValue()

        return 0.0

    def getMinJointValue( self, indx ) :
        if self._isValidIndex( indx ) :
            return self.m_dhentries[ indx ].getMinJointValue()

        return 0.0

    def getMaxJointValue( self, indx ) :
        if self._isValidIndex( indx ) :
            return self.m_dhentries[ indx ].getMaxJointValue()

        return 0.0

    def getRangeJointValue( self, indx ) :
        if self._isValidIndex( indx ) :
            return self.m_dhentries[ indx ].getRangeJointValue()

        return 0.0

    def _buildModel( self ) :
        # Override this method - generate model related stuff here
        pass

    def _isValidIndex( self, indx ) : 
        if indx < 0 or indx >= self.getNumJoints() :
            print 'RDHmodel> Trying to set value for out of range indx ', indx
            return False

        return True