from __future__ import absolute_import, print_function, division

import numpy as np 
from numpy.testing import assert_almost_equal
from . import ransac

## TODO: add so3, 

def procrustes(X,Y,remove_mean=False):
    assert X.shape == Y.shape
    assert X.shape[0] > 1

    if X.shape[1] == 2:
        # cross product
        X3 = np.cross(X[:,0],X[:,1],axis=0).reshape((3,1))
        Y3 = np.cross(Y[:,0],Y[:,1],axis=0).reshape((3,1))
        # hstack and norm
        X = np.hstack((X,X3/np.linalg.norm(X3)))
        Y = np.hstack((Y,Y3/np.linalg.norm(Y3)))

    # get dimension and number of samples
    D,N = X.shape[:2]

    if remove_mean:
        # get the mean value
        mx = np.mean(X,axis=1).reshape(D,1)
        my = np.mean(Y,axis=1).reshape(D,1)
        Xhat = X-mx
        Yhat = Y-my
    else:
        Xhat = X
        Yhat = Y
    # svd
    (U,_,V)=np.linalg.svd((Xhat).dot(Yhat.T))
    Dtmp = np.eye(X.shape[0])
    Dtmp[-1,-1] = np.linalg.det(U.dot(V))

    R_est = U.dot(Dtmp).dot(V)

    if remove_mean:
        t_est = mx - R_est.dot(my)
    else:
        t_est = None

    return (R_est, t_est)

def rotation_matrix_to_axis_angle(R):
    assert R.shape==(3,3)
    assert_almost_equal(np.linalg.det(R),1,err_msg='Invalid R: det is not 1')
    S,V=np.linalg.eig(R)
    k = np.argmin(np.abs(S-1))
    s = S[k]
    assert_almost_equal(s, 1, err_msg='eigen value shoud be close to 1')
    # force to real
    v = np.real(V[:,k])

    vhat = np.array([R[2,1]-R[1,2],R[0,2]-R[2,0],R[1,0]-R[0,1]])
    sintheta = 0.5 * np.dot(v,vhat)
    costheta = 0.5 * (np.trace(R)-1)
    theta = np.arctan2(sintheta,costheta)

    return (v,theta)

def axis_angle_to_rotation_matrix(v, theta):
    if np.abs(theta) < np.spacing(1):
        # if theta is almost zero
        return np.eye(3)
    else:
        # numpy is always row-major
        v = v.reshape(3,1)
        assert_almost_equal(np.linalg.norm(v),1,err_msg='invalid axis, norm is not 1')
        # rodriguez formula
        vhat = np.array([[0,-v[2,0],v[1,0]],\
                        [v[2,0],0,-v[0,0]],\
                        [-v[1,0],v[0,0],0]])
        vvt = np.dot(v,v.T)
        R = np.eye(3)*np.cos(theta) + (1-np.cos(theta))*vvt + vhat * np.sin(theta)
        return R

def quat_to_rotation_matrix(q):
    q=q.flatten()
    assert q.size == 4
    assert_almost_equal(np.linalg.norm(q),1,err_msg='invalid q')
    qq=q**2
    R = np.array([[qq[0] + qq[1] - qq[2] - qq[3], 2*q[1]*q[2] -
2*q[0]*q[3], 2*q[1]*q[3] + 2*q[0]*q[2]],
                [2*q[1]*q[2] + 2*q[0]*q[3], qq[0] - qq[1] + qq[2] -
qq[3], 2*q[2]*q[3] - 2*q[0]*q[1]],
                [2*q[1]*q[3] - 2*q[0]*q[2], 2*q[2]*q[3] + 2*q[0]*q[1],
qq[0] - qq[1] - qq[2] + qq[3]]])
    return R


def integrate_gyro_quaternion(gyro_ts, gyro_data):
    """Integrate angular velocities to rotations

    Parameters
    ---------------
    gyro_ts : ndarray
            Timestamps
    gyro_data : (3, N) ndarray
            Angular velocity measurements

    Returns
    ---------------
    rotations : (4, N) ndarray
            Rotation sequence as unit quaternions (first element scalar)

    """
    #NB: Quaternion q = [a, n1, n2, n3], scalar first
    q_list = np.zeros((gyro_ts.shape[0], 4)) # Nx4 quaternion list
    q_list[0,:] = np.array([1, 0, 0, 0]) # Initial rotation (no rotation)

    # Iterate over all (except first)
    for i in range(1, gyro_ts.size):
        w = gyro_data[i]
        dt = gyro_ts[i] - gyro_ts[i - 1]
        qprev = q_list[i - 1]

        A = np.array([[0,    -w[0],  -w[1],  -w[2]],
                     [w[0],  0,      w[2],  -w[1]],
                     [w[1], -w[2],   0,      w[0]],
                     [w[2],  w[1],  -w[0],   0]])
        qnew = (np.eye(4) + (dt/2.0) * A).dot(qprev)
        qnorm = np.sqrt(np.sum(qnew ** 2))
        qnew /= qnorm
        q_list[i] = qnew

    return q_list

#--------------------------------------------------------------------------

def slerp(q1, q2, u):
    """SLERP: Spherical linear interpolation between two unit quaternions.

    Parameters
    ------------
    q1 : (4, ) ndarray
            Unit quaternion (first element scalar)
    q2 : (4, ) ndarray
            Unit quaternion (first element scalar)
    u : float
            Interpolation factor in range [0,1] where 0 is first quaternion
            and 1 is second quaternion.

    Returns
    -----------
    q : (4,) ndarray
            The interpolated unit quaternion
    """
    q1 = q1.flatten()
    q2 = q2.flatten()
    assert q1.shape == q2.shape
    assert q1.size == 4
    costheta = np.dot(q1, q2)

    if np.isclose(u, 0.):
        return q1
    elif np.isclose(u, 1.):
        return q2
    elif u > 1 or u < 0:
        raise ValueError("u must be in range [0, 1]")

    # Shortest path
    if costheta < 0:
        costheta = -costheta
        q2 = -q2

    # Almost the same, we can return any of them?
    if np.isclose(costheta, 1.0):
        return q1

    theta = np.arccos(costheta)

    f1 = np.sin((1.0 - u)*theta) / np.sin(theta)
    f2 = np.sin(u*theta) / np.sin(theta)
    q = f1*q1 + f2*q2
    q = q / np.sqrt(np.sum(q**2)) # Normalize
    return q

#--------------------------------------------------------------------------

def estimate_rotation_procrustes_ransac(x, y, camera, threshold, inlier_ratio=0.75, do_translation=False):
    """Calculate rotation between two sets of image coordinates using ransac.

    Inlier criteria is the reprojection error of y into image 1.

    Parameters
    -------------------------
    x : array 2xN image coordinates in image 1
    y : array 2xN image coordinates in image 2
    camera : Camera model
    threshold : float pixel distance threshold to accept as inlier
    do_translation : bool Try to estimate the translation as well

    Returns
    ------------------------
    R : array 3x3 The rotation that best fulfills X = RY
    t : array 3x1 translation if do_translation is False
    residual : array pixel distances ||x - xhat|| where xhat ~ KRY (and lens distorsion)
    inliers : array Indices of the points (in X and Y) that are RANSAC inliers
    """
    assert x.shape == y.shape
    assert x.shape[0] == 2

    X = camera.unproject(x)
    Y = camera.unproject(y)
    # reason for stack if purely for using lambda
    data = np.vstack((X, Y, x))
    assert data.shape[0] == 8

    model_func = lambda data: procrustes(data[:3], data[3:6], remove_mean=do_translation)

    def eval_func(model, data):
        Y = data[3:6].reshape(3,-1)
        x = data[6:].reshape(2,-1)
        R, t = model

        Xhat = np.dot(R, Y) if t is None else np.dot(R, Y) + t
        xhat = camera.project(Xhat)
        dist = np.sqrt(np.sum((x-xhat)**2, axis=0))

        return dist

    inlier_selection_prob = 0.99999
    model_points = 2
    ransac_iterations = int(np.log(1 - inlier_selection_prob) / np.log(1-inlier_ratio**model_points))

    model_est, ransac_consensus_idx = ransac.RANSAC(model_func, eval_func, data, model_points, ransac_iterations, threshold, recalculate=True)
    if model_est is not None:
        (R, t) = model_est
        dist = eval_func((R, t), data)
    else:
        dist = None
        R, t = None, None
        ransac_consensus_idx = []

    return R, t, dist, ransac_consensus_idx


def to_rot_matrix(r):
    "Convert combined axis angle vector to rotation matrix"
    theta = np.linalg.norm(r)
    v = r/theta
    R = axis_angle_to_rotation_matrix(v, theta)
    return R


    
