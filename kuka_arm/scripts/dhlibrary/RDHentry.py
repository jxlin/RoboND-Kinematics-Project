
# helper libraries
from mpmath import * # for arbitraty floating point precision operations
from sympy import * # for symbolic operations
import numpy as np

from RDHmath import *

class RJointType :
    NONE = 0
    REVOLUTE = 1
    PRISMATIC = 2

class RDHparams :
    alpha_i_1 = 0
    a_i_1 = 1
    d_i = 2
    theta_i = 3


class RDHentry( object ):

    """
        Object representation of an entry in the DH table
        @param {boolean[]} vfixed Whether or not the joints are fixed or not
        @param {float[]} vvalues Value of the DH params for this entry
        @param {float} minJointValue Minimum value of the joint for this entry
        @param {float} maxJointValue Maximum value of the joint for this entry
        @param {float} jointSign Sign that the joint variable takes in the dh representation
        @param {float} jointOffset Offset applied to the joint variable in the dh representation
    """
    def __init__( self, 
                  vfixed, vvalues, 
                  minJointValue, maxJointValue,
                  jointSign = 1.0, jointOffset = 0.0 ):

        super( RDHentry, self ).__init__()

        assert len( vfixed ) == 4, 'Wrong length for vfixed parameter, should be 4'
        assert len( vvalues ) == 4, 'Wrong length for vvalues parameter, should be 4'
        
        self.m_fixed = vfixed
        self.m_dhparams = vvalues
        self.m_jointSign = jointSign
        self.m_jointOffset = jointOffset
        
        self.m_minJointValue = minJointValue
        self.m_maxJointValue = maxJointValue
        self.m_rangeJointValue = maxJointValue - minJointValue

        self.m_transform = np.eye( 4 )
        self.m_symTransform = eye( 4 )

        self.m_sym_alpha_i_1   = symbols( 'alpha_i_1' )
        self.m_sym_a_i_1       = symbols( 'a_i_1' )
        self.m_sym_d_i         = symbols( 'd_i' )
        self.m_sym_theta_i     = symbols( 'theta_i' )
        self.m_dhparamsSym = { self.m_sym_alpha_i_1 : self.m_dhparams[ RDHparams.alpha_i_1 ],
                               self.m_sym_a_i_1 : self.m_dhparams[ RDHparams.a_i_1 ],
                               self.m_sym_d_i : self.m_dhparams[ RDHparams.d_i ],
                               self.m_sym_theta_i : self.m_dhparams[ RDHparams.theta_i ] }

        self.m_jointType = self._getJointType()

        self._buildSymTransform()
        self._updateTransform()

    def setDHparamValue( self, indx, value ) :
        if indx < 0 or indx > 3 :
            print 'RDHentry> tried to access invalid dh param'
            return

        self.m_dhparams[ indx ] = value
        self._updateTransform()

    def getDHparamValue( self, indx ) :
        if indx < 0 or indx > 3 :
            print 'RDHentry> tried to access invalid dh param'
            return 0.0

        return self.m_dhparams[ indx ]

    def setJointValue( self, value ) :

        if value < self.m_minJointValue :
            print 'RDHmodel> min value reached: ', self.m_minJointValue
            value = self.m_minJointValue
        elif value > self.m_maxJointValue :
            print 'RDHmodel> max value reached: ', self.m_maxJointValue
            value = self.m_maxJointValue

        if self.m_jointType == RJointType.REVOLUTE :
            self.m_dhparams[ RDHparams.theta_i ] = value
        elif self.m_jointType == RJointType.PRISMATIC :
            self.m_dhparams[ RDHparams.d_i ] = value

        self._updateTransform()


    def getJointValue( self ) :

        if self.m_jointType == RJointType.REVOLUTE :
            return self.m_dhparams[ RDHparams.theta_i ]

        elif self.m_jointType == RJointType.PRISMATIC :
            return self.m_dhparams[ RDHparams.d_i ]

    def getDHparamsBehavior( self ) :
        return self.m_fixed

    def getDHparams( self ) :
        return self.m_dhparams

    def getMinJointValue( self ) :
        return self.m_minJointValue

    def getMaxJointValue( self ) :
        return self.m_maxJointValue

    def getRangeJointValue( self ) :
        return self.m_rangeJointValue

    def getTransform( self ) : 
        return self.m_transform

    def getSymTransform( self ) :
        return self.m_symTransform

    def getSymTransformEvaluated( self ) :
        return self.m_symTransform.evalf( subs = self.m_dhparamsSym )
    """
        Return which kind of joint this entry represents
    """
    def _getJointType( self ) :

        if not self.m_fixed[ RDHparams.theta_i ] :
            return RJointType.REVOLUTE

        elif not self.m_fixed[ RDHparams.d_i ] :
            return RJointType.PRISMATIC

        print 'RDHentry> this joint has no degree of freedom :('
        return RJointType.NONE

    """
        Builds the symbolic representation of this entry's world-transform
    """
    def _buildSymTransform( self ):

        self.m_symTransform[0,0] = cos( self.m_sym_theta_i )
        self.m_symTransform[1,0] = sin( self.m_sym_theta_i ) * cos( self.m_sym_alpha_i_1 )
        self.m_symTransform[2,0] = sin( self.m_sym_theta_i ) * sin( self.m_sym_alpha_i_1 )
        self.m_symTransform[3,0] = 0

        self.m_symTransform[0,1] = -sin( self.m_sym_theta_i )
        self.m_symTransform[1,1] = cos( self.m_sym_theta_i ) * cos( self.m_sym_alpha_i_1 )
        self.m_symTransform[2,1] = cos( self.m_sym_theta_i ) * sin( self.m_sym_alpha_i_1 )
        self.m_symTransform[3,1] = 0

        self.m_symTransform[0,2] = 0
        self.m_symTransform[1,2] = -sin( self.m_sym_alpha_i_1 )
        self.m_symTransform[2,2] = cos( self.m_sym_alpha_i_1 )
        self.m_symTransform[3,2] = 0

        self.m_symTransform[0,3] = self.m_sym_a_i_1
        self.m_symTransform[1,3] = -sin( self.m_sym_alpha_i_1 ) * self.m_sym_d_i
        self.m_symTransform[2,3] = cos( self.m_sym_alpha_i_1 ) * self.m_sym_d_i
        self.m_symTransform[3,3] = 1

    """
        Updates both numeric and symbolic world-transforms
    """
    def _updateTransform( self ) :

        _alpha_i_1  = self.m_dhparams[ RDHparams.alpha_i_1 ]
        _a_i_1      = self.m_dhparams[ RDHparams.a_i_1 ]
        _d_i        = self.m_dhparams[ RDHparams.d_i ]
        _theta_i    = self.m_dhparams[ RDHparams.theta_i ]

        if self.m_jointType == RJointType.REVOLUTE :
            _theta_i = _theta_i * self.m_jointSign + self.m_jointOffset

        elif self.m_jointType == RJointType.PRISMATIC :
            _d_i = _d_i * self.m_jointSign + self.m_jointOffset

        self.m_dhparamsSym[ self.m_sym_alpha_i_1 ] = _alpha_i_1
        self.m_dhparamsSym[ self.m_sym_a_i_1 ]     = _a_i_1
        self.m_dhparamsSym[ self.m_sym_d_i ]       = _d_i
        self.m_dhparamsSym[ self.m_sym_theta_i ]   = _theta_i

        # update numeric transform
        self.m_transform[0,0] = np.cos( _theta_i )
        self.m_transform[1,0] = np.sin( _theta_i ) * np.cos( _alpha_i_1 )
        self.m_transform[2,0] = np.sin( _theta_i ) * np.sin( _alpha_i_1 )
        self.m_transform[3,0] = 0

        self.m_transform[0,1] = -np.sin( _theta_i )
        self.m_transform[1,1] = np.cos( _theta_i ) * np.cos( _alpha_i_1 )
        self.m_transform[2,1] = np.cos( _theta_i ) * np.sin( _alpha_i_1 )
        self.m_transform[3,1] = 0

        self.m_transform[0,2] = 0
        self.m_transform[1,2] = -np.sin( _alpha_i_1 )
        self.m_transform[2,2] = np.cos( _alpha_i_1 )
        self.m_transform[3,2] = 0

        self.m_transform[0,3] = _a_i_1
        self.m_transform[1,3] = -np.sin( _alpha_i_1 ) * _d_i
        self.m_transform[2,3] = np.cos( _alpha_i_1 ) * _d_i
        self.m_transform[3,3] = 1

        # update symbolic transform
        self.m_symTransform.subs( self.m_dhparamsSym )