from __future__ import division, print_function, absolute_import

import time
import warnings
import numpy as np 
import matplotlib.pyplot as plt
import scipy.optimize
import logging 
import os
import os.path
logger = logging.getLogger('calib_xiahaa')

from . import rotations, videoslice_new
from . import tracking_new
from . import timesync
from . import ransac
from . import imu_new

PARAM_SOURCE_ORDER = ('user', 'initialized', 'calibrated') # Increasing order of importance
PARAM_ORDER = ('gyro_rate', 'time_offset', 'gbias_x', 'gbias_y', 'gbias_z', 'rot_x', 'rot_y', 'rot_z')

MAX_OPTIMIZATION_TRACKS = 1500
MAX_OPTIMIZATION_FEV = 900
DEFAULT_NORM_C = 3.0

class calibrator(object):
    def __init__(self, video, gyro):
        self.video = video
        self.gyro = gyro
        self.slices = None
        self.params={
            'user':{},
            'initialized':{},
            'calibrated':{},
        }
    
    def initialize(self, gyro_rate, slices=None, skip_estimation=False,is_debug = False):
        self.params['user']['gyro_rate'] = gyro_rate
        logger.info('gyro_rate is provided!')
        print('version-2')
        for p in ('gbias_x', 'gbias_y', 'gbias_z'):
            self.params['initialized'][p] = 0.0
        
        if slices is not None:
            self.slices = slices

        if self.slices is None:
            # self.slices = videoslice_new.from_stream_randomly(self.video)
            # logger.debug("Number of slices: {:d}".format(len(self.slices)))
            if is_debug == True:
                if not os.path.exists(os.path.join(os.getcwd(),'tmp')):
                    os.mkdir(os.getcwd(),'tmp')
                filename=os.path.join(os.getcwd(),'tmp','slices.npy')
                if not os.path.exists(filename):
                    self.slices = videoslice_new.from_stream_randomly(self.video)
                    np.save(filename,self.slices)
                else:
                    self.slices = np.load(filename)
            else:
                self.slices = videoslice_new.from_stream_randomly(self.video)

        if len(self.slices) < 2:
            logger.error("Calibration requires at least 2 video slices to proceed, got %d", len(self.slices))
            raise Exception("Calibration requires at least 2 video slices to proceed, got {:d}".format(len(self.slices)))

        if not skip_estimation:
            time_offset = self.find_initial_offset(is_debug=is_debug)

        R = self.find_initial_rotation()
        if R is None:
            raise Exception("Failed to calculate initial rotation")

    @property
    def parameter(self):
        D={}
        for source in PARAM_SOURCE_ORDER:
            D.update(self.params[source])
        return D
    
    def video_time_to_gyro_sample(self,t):
        gyro_rate = self.parameter['gyro_rate']
        time_offset = self.parameter['time_offset']
        # flow time + time offset = equivalent imu time
        # times rate to get the count
        n = gyro_rate * (t + time_offset)
        n0 = int(np.floor(n))
        tau = n - n0 # for slerp interpolation
        return n0, tau

    def find_initial_offset(self, pyramids = 6, is_debug = False):
        # detect optical flow
        if is_debug == True:
            import os
            if not os.path.exists(os.path.join('./','tmp')):
                os.mkdir(os.path.join('./','tmp'))
            filename = os.path.join('./','tmp','flow.npy')
            if os.path.exists(filename):
                flow = np.load(filename)
            else:
                flow = self.video.flow
                np.save(filename,flow)
        else:
            flow = self.video.flow
        
        if is_debug == True:
            import os
            filename = os.path.join('./','tmp','time_offset.npy')
            if os.path.exists(filename):
                do_estimation = False
            else:
                do_estimation = True
        else:
            do_estimation = True

        # get gyro_rate
        gyro_rate = self.parameter['gyro_rate']
        # initialize virtual timestamps for video sequence
        flow_times = np.arange(len(flow))/self.video.frame_rate
        # initialize virtual timestamps for gyro 
        gyro_times = np.arange(self.gyro.num_samples)/gyro_rate
        if do_estimation:
            # find the offset
            time_offset = timesync.sync_camera_gyro(flow, \
                flow_times, self.gyro.data.T, gyro_times, \
                    levels = pyramids,do_plot=True)
            import os
            filename = os.path.join('./','tmp','time_offset.npy')
            np.save(filename,time_offset)
        else:
            filename = os.path.join('./','tmp','time_offset.npy')
            time_offset = np.load(filename)
            # plot to check
            plt.clf()
            gyro_mag = np.sum(self.gyro.data ** 2, axis=1).T
            plt.subplot(211)
            plt.plot(flow_times, flow, 'r-')
            plt.plot(gyro_times, gyro_mag, 'b-')
            plt.subplot(212)
            plt.plot(flow_times+time_offset, flow, 'r-')
            plt.plot(gyro_times, gyro_mag, 'b-')
            plt.draw()
            plt.waitforbuttonpress(timeout=1000.0)
            plt.close()

        self.params['initialized']['time_offset']=time_offset
       
        return time_offset

    def find_initial_rotation(self):
        if 'time_offset' not in self.parameter:
            raise ValueError('find initial offset first')

        dt = float(1.0/self.parameter['gyro_rate'])
        # integration to get the imu rotation
        q = self.gyro.integrate(dt)
        #
        video_axes = []
        gyro_axes = []

        for _slice in self.slices:
            # estimate rotation here
            # estimate the rotation for slice
            # to make sure within this slice, there is 
            # a significant rotation
            _slice.estimate_rotation(self.video.camera_model,\
                ransac_threshold=7.0)
            if _slice.axis is None:
                continue
            assert _slice.angle > 0

            t1 = _slice.start / self.video.frame_rate
            n1, _ = self.video_time_to_gyro_sample(t1)
            t2 = _slice.end / self.video.frame_rate
            n2, _ = self.video_time_to_gyro_sample(t2)

            try:
                qx = q[n1]
                qy = q[n2]
            except:
                continue

            # to rotation matrix
            Rx = rotations.quat_to_rotation_matrix(qx)
            Ry = rotations.quat_to_rotation_matrix(qy)
            dR = np.dot(Rx.T,Ry)
            v, theta = rotations.rotation_matrix_to_axis_angle(dR)
            if theta < 0:
                v = -v
            # add to gyro rotation axis and video rotation axis
            # here can do sth
            gyro_axes.append(v)
            video_axes.append(_slice.axis)

        if len(gyro_axes) < 2:
            raise Exception('Hand-eye calibration requires >= 2 rotations')
        
        # [0] since we only cares about rotation
        model_func = lambda data: rotations.procrustes(data[:3],data[3:6],remove_mean=False)[0]

        def eval_func(model, data):
            X=data[:3].reshape(3,-1)
            Y=data[3:6].reshape(3,-1)
            R = model
            Xhat = np.dot(R,Y)
            # 3xN * 3xN
            costheta = np.sum(Xhat*X,axis=0)
            theta = np.arccos(costheta)

            return theta
        
        model_points = 2 # select to not use minimal case
        threshold = np.deg2rad(8.0)# tunable
        data = np.vstack((np.array(video_axes).T, np.array(gyro_axes).T))
        
        R, _ = ransac.adaRANSAC(model_func,\
            eval_func,data,model_points,threshold,recalculate=True)
        
        n, theta = rotations.rotation_matrix_to_axis_angle(R)
        rx, ry, rz = theta*n

        self.params['initialized']['rot_x'] = rx
        self.params['initialized']['rot_y'] = ry
        self.params['initialized']['rot_z'] = rz

        return R

    def calibrate(self, max_tracks=MAX_OPTIMIZATION_TRACKS,\
        max_eval = MAX_OPTIMIZATION_FEV, norm_c = DEFAULT_NORM_C):
        # initialization optimization vectors
        x0 = np.array([self.parameter[param] for param in PARAM_ORDER])
        # for each slice, use tracked points to est R and reidentify inliers
        available_tracks = np.sum([len(s.inliers) for s in self.slices])
        if available_tracks < max_tracks:
            warnings.warn('less valid tracks')
            max_tracks = available_tracks
        
        # resampling
        slice_sample_idxs = videoslice_new.fill_sampling(self.slices, max_tracks)

        # arguments
        func_args = (self.slices, slice_sample_idxs, \
            self.video.camera_model, self.gyro, norm_c)
        self.slice_sample_idxs = slice_sample_idxs

        start_time = time.time()
        leastsq_result = scipy.optimize.leastsq(\
            optimization_func,
            x0, args=func_args,full_output=True,
            ftol=1e-10,
            xtol=1e-10, maxfev=max_eval)# max_eval is the maximum number of iterations
        elapsed = time.time() - start_time
        # unpack result
        x,covx,infodict,mesg,ier = leastsq_result
        print('Optimization Completed in {:.1f} seconds and {:d} function evaluations. ier={},mesg={}'.format(elapsed,infodict['nfev'],ier,mesg))
        if ier in (1,2,3,4):
            for pname, val in zip(PARAM_ORDER,x):
                self.params['calibrated'][pname] = val
            return self.parameter
        else:
            raise Exception('Calibration Failed')

# compute sample index and dt to nearest integer index
def sample_at_time(t,rate):
    s = t*rate - 0.5
    n = int(np.floor(s))
    tau = s - n
    return n, tau

# robust kernel for suppress the influence of outliers
def robust_norm(r,c):
    return r / (1+(np.abs(r)/c))

def optimization_func(x, slices, slice_sample_idxs, camera, gyro, norm_c):
    # unpack variables
    Fg, offset, gbias_x, gbias_y, gbias_z, rot_x, rot_y, rot_z = x
    # pack to a numpy array
    gyro_bias = np.array([gbias_x,gbias_y,gbias_z])
    # form rotation matrix
    v = np.array([rot_x,rot_y,rot_z])
    theta = np.linalg.norm(v)
    v = v/theta
    R_g2c = rotations.axis_angle_to_rotation_matrix(v,theta)
    # gyro dt using updated rate
    Tg = float(1.0/Fg) 
    # dt per line scan
    row_delta = camera.readout / camera.rows
    # final errors
    errors = []
    # margin of integration is amount of gyro samples per frame
    integration_margin = int(np.ceil(Fg*camera.readout))

    for _slice, sample_idxs in zip(slices,slice_sample_idxs):
        # now we have each slice and the inliers index we need to sample for this slice
        if len(sample_idxs) < 1:
            continue
        
        # synchronized time for the first frame in the slice
        t_start = _slice.start / camera.frame_rate + offset
        # synchonized time for the last frame in the slice
        t_end = _slice.end / camera.frame_rate + offset
        # index for synchronized gyro to slice start
        slice_start,_ = sample_at_time(t_start, Fg)
        # index for synchronized gyro to slice end
        slice_end,_ = sample_at_time(t_end, Fg) 
        #
        slice_end+=1

        # gyro samples to integrate within
        integration_start = slice_start
        # leave a margin since the initial offset is based on GS property
        integration_end = slice_end + integration_margin

        # handle extreme cases
        if integration_start < 0 or integration_end >= gyro.num_samples:
            # just skip
            continue
        else:
            gyro_part=gyro.data[integration_start:integration_end+1]
        # remove bias
        gyro_part_unbiased = gyro_part + gyro_bias
        # use updated integration time
        q = imu_new.integrate_gyro_quaternion_uniform(gyro_part_unbiased,Tg)
        
        for track in _slice.points[sample_idxs]:
            x = track[0]
            y = track[-1]

            # get row time
            tx = t_start + x[1] * row_delta
            ty = t_end + y[1] * row_delta

            # sample index and interpolation value for point correspondence
            nx, taux = sample_at_time(tx, Fg)# nearest but lower
            ny, tauy = sample_at_time(ty, Fg)

            # interpolation using slerp
            a = nx - integration_start # index relative to the buffer
            b = ny - integration_start
            qx = rotations.slerp(q[a],q[a+1],taux)
            qy = rotations.slerp(q[b],q[b+1],tauy)

            # to rotation matrix
            Rx = rotations.quat_to_rotation_matrix(qx)
            Ry = rotations.quat_to_rotation_matrix(qy)
            dR = np.dot(Rx.T,Ry)
            # TODO: what is the definition of the rotation exactly
            R = R_g2c.dot(dR).dot(R_g2c.T)
            Y = camera.unproject(y)
            Xhat = np.dot(R,Y)
            xhat = camera.project(Xhat)

            # compute err and append to errrors
            err = x - xhat.flatten()
            errors.extend(err.flatten())

            # symmetric errors
            dR1 = np.dot(Ry.T, Rx)
            R1 = R_g2c.dot(dR1).dot(R_g2c.T)
            X = camera.unproject(x)
            Yhat = np.dot(R1,X)
            yhat = camera.project(Yhat)

            err = y - yhat.flatten()
            errors.extend(err.flatten())
    if not errors:
        raise ValueError('No residuals')
    # apply robust norm
    robust_errors = robust_norm(np.array(errors),norm_c)

    return robust_errors

