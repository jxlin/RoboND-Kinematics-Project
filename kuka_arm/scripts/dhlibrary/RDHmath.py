
import numpy as np

def quat2numpy( quat ) :
    _res = np.zeros( 4 )
    _res[0] = quat.x
    _res[1] = quat.y
    _res[2] = quat.z
    _res[3] = quat.w

    return _res

def vec3d2numpy( vec3 ) :
    _res = np.zeros( 3 )
    _res[0] = vec3.x
    _res[1] = vec3.y
    _res[2] = vec3.z

    return _res

def rotationMatrix2euler( mat ) :
    # Assuming Tait-Bryan, xyz extrinsic

    _res = np.zeros( ( 3, 1 ) )

    _r11 = mat[0,0]
    _r21 = mat[1,0]
    _r31 = mat[2,0]
    _r32 = mat[2,1]
    _r33 = mat[2,2]

    _res[0,0] = np.arctan2( _r32, _r33 )
    _res[1,0] = np.arctan2( -_r32, np.sqrt( _r11 ** 2 + _r21 ** 2 ) )
    _res[2,0] = np.arctan2( _r21, _r11 )

    return _res

def euler2rotationMatrix( rpy ) :
    # Assuming Tait-Bryan, xyz extrinsic

    _res = np.eye( 4 )

    _c1 = np.cos( rpy[0,0] )
    _s1 = np.sin( rpy[0,0] )

    _c2 = np.cos( rpy[1,0] )
    _s2 = np.sin( rpy[1,0] )

    _c3 = np.cos( rpy[2,0] )
    _s3 = np.sin( rpy[2,0] )

    _res[0,0] = _c1 * _c2;
    _res[1,0] = _c2 * _s1;
    _res[2,0] = -_s2;

    _res[0,1] = _c1 * _s2 * _s3 - _c3 * _s1;
    _res[1,1] = _c1 * _c3 + _s1 * _s2 * _s3;
    _res[2,1] = _c2 * _s3;

    _res[0,2] = _s1 * _s3 + _c1 * _c3 * _s2;
    _res[1,2] = _c3 * _s1 * _s2 - _c1 * _s3;
    _res[2,2] = _c2 * _c3;

    return _res

def rotationX( angle ) :

    _res = np.eye( 4 )

    _c = np.cos( angle )
    _s = np.sin( angle )

    # _res[0,0] = 1
    # _res[1,0] = 0
    # _res[2,0] = 0

    _res[0,1] = 0
    _res[1,1] = _c
    _res[2,1] = _s

    _res[0,2] = 0
    _res[1,2] = -_s
    _res[2,2] = _c

    return _res

def rotationY( angle ) :

    _res = np.eye( 4 )

    _c = np.cos( angle )
    _s = np.sin( angle )

    _res[0,0] = _c
    _res[1,0] = 0
    _res[2,0] = -_s

    # _res[0,1] = 0
    # _res[1,1] = 1
    # _res[2,1] = 0

    _res[0,2] = _s
    _res[1,2] = 0
    _res[2,2] = _c

    return _res

def rotationZ( angle ) :

    _res = np.eye( 4 )

    _c = np.cos( angle )
    _s = np.sin( angle )

    _res[0,0] = _c
    _res[1,0] = _s
    _res[2,0] = 0

    _res[0,1] = -_s
    _res[1,1] = _c
    _res[2,1] = 0

    # _res[0,2] = 0
    # _res[1,2] = 0
    # _res[2,2] = 1

    return _res

def translation( vec ) :

    _res = np.eye( 4 )

    _res[0,3] = vec[0,0]
    _res[1,3] = vec[1,0]
    _res[2,3] = vec[2,0]

    return _res