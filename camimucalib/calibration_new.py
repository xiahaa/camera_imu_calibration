from __future__ import division, print_function, absolute_import

import time
import warnings
import numpy as np 
import matplotlib.pyplot as plt
import scipy.optimize
import logging 
import os
logger = logging.getLogger('calib_xiahaa')

from . import rotations, videoslice_new
from . import tracking_new
from . import timesync
from . import ransac

PARAM_SOURCE_ORDER = ('user', 'initialized', 'calibrated') # Increasing order of importance
PARAM_ORDER = ('gyro_rate', 'time_offset', 'gbias_x', 'gbias_y', 'gbias_z', 'rot_x', 'rot_y', 'rot_z')

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
        
        inlier_selection_prob = 0.99
        model_points = 2 # select to not use minimal case
        inlier_ratio = 0.5
        threshold = np.deg2rad(8.0)# tunable
        max_iterations = int(np.log(1-inlier_selection_prob)/np.log(1-inlier_ratio**model_points))
        data = np.vstack((np.array(video_axes).T, np.array(gyro_axes).T))
        
        R, _ = ransac.RANSAC(model_func,\
            eval_func,data,model_points,max_iterations,threshold,recalculate=True)
        
        n, theta = rotations.rotation_matrix_to_axis_angle(R)
        rx, ry, rz = theta*n

        self.params['initialized']['rot_x'] = rx
        self.params['initialized']['rot_y'] = ry
        self.params['initialized']['rot_z'] = rz

        return R
