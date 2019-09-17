# -*- coding: utf-8 -*-
from __future__ import division, print_function, absolute_import

import numpy as np
import struct
import re
import scipy.io

from . import rotations

"""
IMU module
"""
__author__ = "Hannes Ovrén"
__copyright__ = "Copyright 2013, Hannes Ovrén"
__license__ = "GPL"
__email__ = "hannes.ovren@liu.se"



"""
Created on Wed Mar  5 10:24:38 2014

@author: hannes
"""
def post_process_L3G4200D_data(data, do_plot=False):
    def notch(Wn, bandwidth):
        f = Wn/2.0
        R = 1.0 - 3.0*(bandwidth/2.0)
        K = ((1.0 - 2.0*R*np.cos(2*np.pi*f) + R**2)/(2.0 -
        2.0*np.cos(2*np.pi*f)))
        b,a = np.zeros(3),np.zeros(3)
        a[0] = 1.0
        a[1] = - 2.0*R*np.cos(2*np.pi*f)
        a[2] = R**2
        b[0] = K
        b[1] = -2*K*np.cos(2*np.pi*f)
        b[2] = K
        return b,a

    # Remove strange high frequency noise and bias
    b,a = notch(0.8, 0.03)
    data_filtered = np.empty_like(data)
    from scipy.signal import filtfilt
    for i in range(3):
        data_filtered[i] = filtfilt(b, a, data[i])

    if do_plot:
        from matplotlib.pyplot import subplot, plot, specgram, title
        # Plot the difference
        ax = None
        for i in range(3):
            if ax is None:
                ax = subplot(5,1,i+1)
            else:
                subplot(5,1,i+1, sharex=ax, sharey=ax)
            plot(data[i])
            plot(data_filtered[i])
            title(['x','y','z'][i])
        #subplot(5,1,4)
        #specgram(data[0])
        #title("Specgram of biased X")
        #subplot(5,1,5)
        #specgram(data_filtered[0])
        #title("Specgram of filtered unbiased X")

    return data_filtered

def integrate_gyro_quaternion_uniform(gyro_data, dt, initial=None):
    #NB: Quaternion q = [a, n1, n2, n3], scalar first
    N = gyro_data.shape[0]
    q_list = np.empty((N, 4)) # Nx4 quaternion list
    dt_half = dt / 2.0

    # Initial rotation
    if initial is None:
        q0 = 1.0
        q1 = q2 = q3 = 0.0
    else:
        q0, q1, q2, q3 = initial

    for i in range(N):
        wx = gyro_data[i,0]
        wy = gyro_data[i,1]
        wz = gyro_data[i,2]

        q_list[i, 0] = q0 + dt_half * (-wx*q1 -wy*q2 -wz*q3)
        q_list[i, 1] = q1 + dt_half * (q0*wx + q2*wz - wy*q3)
        q_list[i, 2] = q2 + dt_half * (wy*q0 -wz*q1 + wx*q3)
        q_list[i, 3] = q3 + dt_half * (wz*q0 + wy*q1 -wx*q2)

        # Normalize
        qnorm = np.sqrt(q_list[i, 0]**2 + q_list[i, 1]**2 + q_list[i, 2]**2 + q_list[i, 3]**2)
        for j in range(4):
            q_list[i, j] /= qnorm

        # New prev values
        q0 = q_list[i, 0]
        q1 = q_list[i, 1]
        q2 = q_list[i, 2]
        q3 = q_list[i, 3]
    return q_list

class IMU(object):
    """
    Defines an IMU (currently only gyro)
    """
    def __init__(self):
        self.integrated = []
        self.gyro_data = []
        self.timestamps = []

    @property
    def rate(self):
        """Get the sample rate in Hz.

        Returns
        ---------
        rate : float
                The sample rate, in Hz, calculated from the timestamps
        """
        N = len(self.timestamps)
        t = self.timestamps[-1] - self.timestamps[0]
        rate = 1.0 * N / t
        return rate

    def zero_level_calibrate(self, duration, t0=0.0):
        """Performs zero-level calibration from the chosen time interval.

        This changes the previously lodaded data in-place.

        Parameters
        --------------------
        duration : float
                Number of timeunits to use for calibration
        t0 : float
                Starting time for calibration

        Returns
        ----------------------
        gyro_data : (3, N) float ndarray
                The calibrated data (note that it is also changed in-place!)
        """

        t1 = t0 + duration
        indices = np.flatnonzero((self.timestamps >= t0) & (self.timestamps <= t1))
        m = np.mean(self.gyro_data[:, indices], axis=1)
        self.gyro_data -= m.reshape(3,1)

        return self.gyro_data

    def gyro_data_corrected(self, pose_correction=np.eye(3)):
        """Get relative pose corrected data.

        Parameters
        -------------
        pose_correction : (3,3) ndarray, optional
                Rotation matrix that describes the relative pose between the IMU and something else (e.g. camera).

        Returns
        ---------------
        gyro_data : (3, N) ndarray
                The relative pose corrected data.
        """
        return pose_correction.dot(self.gyro_data)

    def integrate(self, pose_correction=np.eye(3), uniform=True):
        """Integrate angular velocity measurements to rotations.

        Parameters
        -------------
        pose_correction : (3,3) ndarray, optional
                Rotation matrix that describes the relative pose between the IMU and something else (e.g. camera).
        uniform : bool
                If True (default), assume uniform sample rate. This will use a faster integration method.
        Returns
        -------------
        rotations : (4, N) ndarray
                Rotations as unit quaternions with scalar as first element.
        """

        if uniform:
            dt = float(self.timestamps[1]-self.timestamps[0]) # Must be python float for fastintegrate to work
            return integrate_gyro_quaternion_uniform(self.gyro_data_corrected, dt)
        else:
            N = len(self.timestamps)
            integrated = np.zeros((4, N))
            integrated[:,0] = np.array([1, 0, 0, 0]) # Initial rotation (no rotation)

            # Iterate over all
            for i in range(1, len(self.timestamps)):
                w = pose_correction.dot(self.gyro_data[:, i]) # Change to correct coordinate frame
                dt = float(self.timestamps[i] - self.timestamps[i - 1])
                qprev = integrated[:, i - 1].flatten()

                A = np.array([[0,    -w[0],  -w[1],  -w[2]],
                             [w[0],  0,      w[2],  -w[1]],
                             [w[1], -w[2],   0,      w[0]],
                             [w[2],  w[1],  -w[0],   0]])
                qnew = (np.eye(4) + (dt/2.0) * A).dot(qprev)
                qnorm = np.sqrt(np.sum(qnew ** 2))
                qnew = qnew / qnorm if qnorm > 0 else 0
                integrated[:, i] = qnew
                #print "%d, %s, %s, %s, %s" % (i, w, dt, qprev, qnew)
            return integrated

    @staticmethod
    def rotation_at_time(t, timestamps, rotation_sequence):
        """Get the gyro rotation at time t using SLERP.

        Parameters
        -----------
        t : float
                The query timestamp.
        timestamps : array_like float
                List of all timestamps
        rotation_sequence : (4, N) ndarray
                Rotation sequence as unit quaternions with scalar part as first element.

        Returns
        -----------
        q : (4,) ndarray
                Unit quaternion representing the rotation at time t.
        """
        idx = np.flatnonzero(timestamps >= (t - 0.0001))[0]
        t0 = timestamps[idx - 1]
        t1 = timestamps[idx]
        tau = (t - t0) / (t1 - t0)

        q1 = rotation_sequence[:, idx - 1]
        q2 = rotation_sequence[:, idx]
        q = rotations.slerp(q1, q2, tau)
        return q