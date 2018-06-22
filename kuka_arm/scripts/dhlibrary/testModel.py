
# helper libraries
from mpmath import * # for arbitraty floating point precision operations
from sympy import * # for symbolic operations
import numpy as np

# Helper functionality to represent each DH joint
from RDHentry import *
from RDHmath import *
# To inherit from
from RDHmodelKukaKR210 import *

# initialize pretry printing
init_printing()

# create kuka kr210 model
_model = RDHmodelKukaKR210()
_model.updateModel()

_totalTransform = eye( 4 )

# print individual transforms
for i in range( _model.getNumJoints() ) :
    # _symTransform = _model.getSymJointTransform( i )
    _symTransform = _model.getSymJointTransformEvaluated( i )

    if _symTransform is None :
        print 'ERROR> something went wrong, this transform does not exists'
        continue

    _totalTransform = _totalTransform * _symTransform
    # simplify( _totalTransform )

    print 'transform ', i, ' - ', ( i + 1 ), ' :'
    pprint( _symTransform )
    
# # print total transform
print 'total transform :'
pprint( _totalTransform )

# # compensate end effector pose
# _eefCompensation = rot_axis3( pi ) * rot_axis2( -pi / 2 )

# # print end effector transform
# _eefTransform = _totalTransform * _eefCompensation
# print 'end effector total transform :'
# print _eefTransform
