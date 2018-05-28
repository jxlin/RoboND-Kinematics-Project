
# helper libraries
from mpmath import * # for arbitraty floating point precision operations
from sympy import * # for symbolic operations
import numpy as np

# Helper functionality to represent each DH joint
from RDHentry import *
from RDHmath import *
# To inherit from
from RDHmodel import *



class RDHmodelKukaKR210( RDHmodel ) :

    def __init__( self ) :

        self.m_eeOffset = np.array( [ [0.0], [0.0], [0.193 + 0.11] ] )

        self.m_ikEEPosRef = np.zeros( ( 3, 1 ) )
        self.m_ikWCPosRef = np.zeros( ( 3, 1 ) )

        self.m_ikEErotMat = np.eye( 4 )
        self.m_ikWCrotMat = np.eye( 4 )

        self.m_ikEEtoWCrot = np.dot( rotationZ( np.pi ), rotationY( -0.5 * np.pi ) )
        self.m_ikEEtoWCinvrot = np.transpose( self.m_ikEEtoWCrot )

        self.m_R_0_3     = np.eye( 4 )
        self.m_R_0_3_inv = np.eye( 4 )
        self.m_R_3_6     = np.eye( 4 )

        super( RDHmodelKukaKR210, self ).__init__()

    def _buildModel( self ) :
        print 'Building kuka kr210 model ...'

        self.m_dhentries.append( RDHentry( [ True, True, True, False ],
                                           [ 0.0, 0.0, 0.75, 0.0 ],
                                           -np.pi, np.pi ) )

        self.m_dhentries.append( RDHentry( [ True, True, True, False ],
                                           [ -0.5 * np.pi, 0.35, 0.0, 0.0 ],
                                           -np.pi, np.pi, 1, -0.5 * np.pi ) )

        self.m_dhentries.append( RDHentry( [ True, True, True, False ],
                                           [ 0.0, 1.25, 0.0, 0.0 ],
                                           -np.pi, np.pi ) )

        self.m_dhentries.append( RDHentry( [ True, True, True, False ],
                                           [ -0.5 * np.pi, -0.054, 1.5, 0.0 ],
                                           -np.pi, np.pi ) )

        self.m_dhentries.append( RDHentry( [ True, True, True, False ],
                                           [ 0.5 * np.pi, 0.0, 0.0, 0.0 ],
                                           -np.pi, np.pi ) )

        self.m_dhentries.append( RDHentry( [ True, True, True, False ],
                                           [ -0.5 * np.pi, 0.0, 0.0, 0.0 ],
                                           -np.pi, np.pi ) )

        self.m_endEffectorCompensation = translation( self.m_eeOffset )
        self.m_endEffectorCompensation = np.dot( self.m_endEffectorCompensation, self.m_ikEEtoWCrot )

        print 'Done'

    def _computeAngleCosineLaw( self, l1, l2, lx, computeId = 'none' ) :

        _num = l1 ** 2 + l2 ** 2 - lx ** 2
        _den = 2 * l1 * l2

        _cosx = _num / _den

        # Check if in range
        if _cosx > 1 or _cosx < -1 :
            print 'Warning> out of domain error computing angle by cosine law ******'
            print '_num: ', _num
            print '_den: ', _den
            print '_cosx: ', _cosx
            print 'computeId: ', computeId
            print '*****************************************************************'

        return np.arccos( _cosx )

    def inverse( self, xyz, rpy, applyToModel = True ) :

        # Extract info from dh table
        _d1 = self.m_dhentries[0].getDHparamValue( RDHparams.d_i )
        _a1 = self.m_dhentries[1].getDHparamValue( RDHparams.a_i_1 )
        _a2 = self.m_dhentries[2].getDHparamValue( RDHparams.a_i_1 )
        _a3 = self.m_dhentries[3].getDHparamValue( RDHparams.a_i_1 )
        _d4 = self.m_dhentries[3].getDHparamValue( RDHparams.d_i )

        # Copy the EEffector position
        self.m_ikEEPosRef = xyz.copy()
        # Compute rotation matrix of the end effector
        self.m_ikEErotMat = euler2rotationMatrix( rpy )

        # Compute wrist orientation - last frame
        self.m_ikWCrotMat = np.dot( self.m_ikEErotMat, self.m_ikEEtoWCinvrot )

        # Compute wrist position - last frame
        # From : r_ee = r_wc + R_0_6 * [ 0, 0, d_ee ]
        # we have -> r_wc = r_ee - R_0_6 * [ 0,0,d_ee ]
        self.m_ikWCPosRef = self.m_ikEEPosRef - np.dot( self.m_ikWCrotMat[0:3,0:3], self.m_eeOffset )

        #### Compute q1, q2 and q3 using some trigonometry
        ## Compute q1
        _q1 = np.arctan2( self.m_ikWCPosRef[1,0], self.m_ikWCPosRef[0,0] )
        ## Compute q2 and q3
        # Compute intermediate variables from geometric relationships
        _phi1 = np.arctan2( _a1, _d1 )
        _phi2 = np.arctan2( self.m_ikWCPosRef[2,0],
                            np.sqrt( self.m_ikWCPosRef[0,0] ** 2 +
                                     self.m_ikWCPosRef[1,0] ** 2 ) )
        _phi3 = 0.5 * np.pi - _phi1 - _phi2
        _phi4 = 0.5 * np.pi - _phi1
        # Compute intermediate variables for cosine law application in ...
        # triangle 0-2-Wc
        _l1 = np.sqrt( _d1 ** 2 + _a1 ** 2 )
        _l2 = np.linalg.norm( self.m_ikWCPosRef )

        _l3 = np.sqrt( _l1 ** 2 + _l2 ** 2 - 2 * _l1 * _l2 * np.cos( _phi3 ) )
        _phi5 = self._computeAngleCosineLaw( _l1, _l3, _l2, 'phi5' )
        # _phi5 = np.arccos( ( _l1 * _l1 + _l3 * _l3 - _l2 * _l2 ) / 
        #                    ( 2 * _l1 * _l3 ) )

        # Compute intermediate variables for cosine law application in ...
        # triangle 2-3-Wc
        _l4 = np.sqrt( _d4 * _d4 + _a3 * _a3 )

        _phi8 = np.arctan2( np.abs( _a3 ), _d4 )
        _phi6 = self._computeAngleCosineLaw( _l3, _a2, _l4, 'phi6' )
        _phi7 = self._computeAngleCosineLaw( _a2, _l4, _l3, 'phi7' )
        # _phi6 = np.arccos( ( _l3 * _l3 + _a2 * _a2 - _l4 * _l4  ) / 
        #                    ( 2 * _l3 * _a2 ) )
        # _phi7 = np.arccos( ( _a2 * _a2 + _l4 * _l4 - _l3 * _l3 ) /
        #                    ( 2 * _a2 * _l4 ) )


        # Compute q2 and q3 with some angles' sums
        _q2 = 1.5 * np.pi - _phi4 - _phi5 - _phi6
        _q3 = 0.5 * np.pi - _phi7 - _phi8

        #### Compute q4, q5 and q6 from the total rotation matrix
        ## Compute R_3_6 ( rotation matrix of frame 6 respect to 3 ) ...
        ## using : R_0_3 * R_3_6 = R_0_6 -> R_3_6 = R_0_3^-1 * R_0_6
        # From T_0_3, R_0_3 is given by :
        _c1 = np.cos( _q1 )
        _s1 = np.sin( _q1 )
        _c23 = np.cos( _q2 + _q3 )
        _s23 = np.sin( _q2 + _q3 )

        self.m_R_0_3[0,0] = _c1 * _s23
        self.m_R_0_3[1,0] = _s1 * _s23
        self.m_R_0_3[2,0] = _c23

        self.m_R_0_3[0,1] = _c1 * _c23
        self.m_R_0_3[1,1] = _s1 * _c23
        self.m_R_0_3[2,1] = -_s23

        self.m_R_0_3[0,2] = -_s1
        self.m_R_0_3[1,2] = _c1
        self.m_R_0_3[2,2] = 0

        # From R_3_6 = R_0_3^-1 * R_0_6
        self.m_R_0_3_inv = np.transpose( self.m_R_0_3 )
        self.m_R_3_6 = np.dot( self.m_R_0_3_inv, self.m_ikWCrotMat )

        ## Compute q4, q5 and q6 from R_3_6 using the FK decomposition for T_3_6
        _r33 = self.m_R_3_6[2,2]
        _r13 = self.m_R_3_6[0,2]
        _r23 = self.m_R_3_6[1,2]
        _r22 = self.m_R_3_6[1,1]
        _r21 = self.m_R_3_6[1,0]

        _q4 = np.arctan2( _r33, -_r13 )
        _q5 = np.arctan2( np.sqrt( _r13 * _r13 + _r33 * _r33 ), _r23 )
        _q6 = np.arctan2( -_r22, _r21 )

        # Send the joints to the model
        _joints = [ _q1, _q2, _q3, _q4, _q5, _q6 ]

        if applyToModel :

            for i in range( self.getNumJoints() ) :

                if ( np.isnan( _joints[i] ) ) :
                    print 'warning, non reachable'

                    # Print every computation for debug

                    # DEBUG: Dump info for debug

                    print 'xyz: ', xyz
                    print 'rpy: ', rpy

                    print '_d1: ', _d1
                    print '_a1: ', _a1
                    print '_a2: ', _a2
                    print '_a3: ', _a3
                    print '_d4: ', _d4

                    print 'm_ikEEPosRef: ', self.m_ikEEPosRef
                    print 'm_ikEErotMat: ', self.m_ikEErotMat
                    print 'm_ikWCrotMat: ', self.m_ikWCrotMat
                    print 'm_ikWCPosRef: ', self.m_ikWCPosRef

                    print '_q1: ', _q1

                    print '_l1: ', _l1
                    print '_l2: ', _l2
                    print '_l3: ', _l3
                    print '_l4: ', _l4
                    
                    print '_phi1: ', _phi1
                    print '_phi2: ', _phi2
                    print '_phi3: ', _phi3
                    print '_phi4: ', _phi4
                    print '_phi4: ', _phi4
                    print '_phi5: ', _phi5


                    return None

                self.setJointValue( i, _joints[i] )

            self.updateModel()

        return _joints