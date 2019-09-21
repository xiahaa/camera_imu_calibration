# -*- coding: utf-8 -*-
from __future__ import division, print_function, absolute_import

"""
Camera module
"""
__author__ = "Hannes Ovrén"
__copyright__ = "Copyright 2013, Hannes Ovrén"
__license__ = "GPL"
__email__ = "hannes.ovren@liu.se"

import os
import glob
import logging
logger = logging.getLogger()

import numpy as np
import cv2
import scipy.interpolate

class CameraModel(object):
    """Class that describes a camera model

    This encapsulates knowledge of a specific camera,
    i.e. its parameters and how the image is formed.

    Note that all cameras are assumed to be rolling shutter cameras.
    """
    def __init__(self, image_size, frame_rate, readout):
        """Create camera model

        Parameters
        -----------------
        image_size : tuple (rows, columns)
            The size of the image in pixels
        frame_rate : float
            The frame rate of the camera
        readout : float
            Rolling shutter readout time. Set to 0 for global shutter cameras.
        """
        self.image_size = image_size
        self.frame_rate = frame_rate
        self.readout = readout

    @property
    def rows(self):
        return self.image_size[1]

    @property
    def columns(self):
        return self.image_size[0]

    def project(self, points):
        """Project 3D points to image coordinates.

        This projects 3D points expressed in the camera coordinate system to image points.

        Parameters
        --------------------
        points : (3, N) ndarray
            3D points

        Returns
        --------------------
        image_points : (2, N) ndarray
            The world points projected to the image plane
        """
        raise NotImplementedError("Class {} does not implement project()".format(self.__class__.__name__))

    def unproject(self, image_points):
        """Find (up to scale) 3D coordinate of an image point

        This is the inverse of the `project` function.
        The resulting 3D points are only valid up to an unknown scale.

        Parameters
        ----------------------
        image_points : (2, N) ndarray
            Image points

        Returns
        ----------------------
        points : (3, N) ndarray
            3D coordinates (valid up to scale)
        """
        raise NotImplementedError("Class {} does not implement unproject()".format(self.__class__.__name__))

class AtanCameraModel(CameraModel):
    """atan camera model

    This implements the camera model of Devernay and Faugeras ([1]_) using the simplified form in [2]_.

    References
    -----------------------
    ..  [1] F. Devernay and O. Faugeras, “Straight lines have to be straight: Au- tomatic calibration and removal of
        distortion from scenes of structured environments,” Machine Vision and Applications, vol. 13, 2001.

    ..  [2] Johan Hedborg and Björn Johansson. "Real time camera ego-motion compensation and lens undistortion on GPU."
        Technical report, Linköping University, Department of Electrical Engineering, Sweden, 2007
    """
    def __init__(self, image_size, frame_rate, readout, camera_matrix, dist_center, dist_param):
        """Create model

        Parameters
        ------------------------
        image_size : tuple (rows, columns)
            The size of the image in pixels
        frame_rate : float
            The frame rate of the camera
        readout : float
            Rolling shutter readout time. Set to 0 for global shutter cameras.
        camera_matrix : (3, 3) ndarray
            The internal camera calibration matrix
        dist_center : (2, ) ndarray
            Distortion center in pixels
        dist_param : float
            Distortion parameter
        """
        super(AtanCameraModel, self).__init__(image_size, frame_rate, readout)
        self.camera_matrix = camera_matrix
        self.inv_camera_matrix = np.linalg.inv(self.camera_matrix)
        self.wc = dist_center
        self.lgamma = dist_param

    def invert(self, points):
        """Invert the distortion

        Parameters
        ------------------
        points : ndarray
            Input image points

        Returns
        -----------------
        ndarray
            Undistorted points
        """
        X = points if not points.ndim == 1 else points.reshape((points.size, 1))
        wx, wy = self.wc

        # Switch to polar coordinates
        rn = np.sqrt((X[0,:] - wx)**2 + (X[1,:] - wy)**2)
        phi = np.arctan2(X[1,:] - wy, X[0,:]-wx)
        # 'atan' method
        r = np.tan(rn * self.lgamma) / self.lgamma

        # Switch back to rectangular coordinates
        Y = np.ones(X.shape)
        Y[0,:] = wx + r * np.cos(phi)
        Y[1,:]= wy + r * np.sin(phi)
        return Y

    def apply(self, points):
        """Apply the distortion

        Parameters
        ---------------------
        points : ndarray
            Input image points

        Returns
        -----------------
        ndarray
            Distorted points
        """
        X = points if not points.ndim == 1 else points.reshape((points.size, 1))

        wx, wy = self.wc

        # Switch to polar coordinates
        rn = np.sqrt((X[0,:] - wx)**2 + (X[1,:] - wy)**2)
        phi = np.arctan2(X[1,:] - wy, X[0,:] - wx)

        r = np.arctan(rn * self.lgamma) / self.lgamma

        # Switch back to rectangular coordinates
        Y = np.ones(X.shape)
        Y[0,:] = wx + r * np.cos(phi)
        Y[1,:] = wy + r * np.sin(phi)

        return Y

    def project(self, points):
        """Project 3D points to image coordinates.

        This projects 3D points expressed in the camera coordinate system to image points.

        Parameters
        --------------------
        points : (3, N) ndarray
            3D points

        Returns
        --------------------
        image_points : (2, N) ndarray
            The world points projected to the image plane
        """
        K = self.camera_matrix
        XU = points
        XU = XU / np.tile(XU[2], (3,1))
        X = self.apply(XU)
        x2d = np.dot(K, X)
        return from_homogeneous(x2d)

    def unproject(self, image_points):
        """Find (up to scale) 3D coordinate of an image point

        This is the inverse of the `project` function.
        The resulting 3D points are only valid up to an unknown scale.

        Parameters
        ----------------------
        image_points : (2, N) ndarray
            Image points

        Returns
        ----------------------
        points : (3, N) ndarray
            3D coordinates (valid up to scale)
        """
        Ki = self.inv_camera_matrix
        X = np.dot(Ki, to_homogeneous(image_points))
        X = X / X[2]
        XU = self.invert(X)
        return XU


class OpenCVCameraModel(CameraModel):
    """OpenCV camera model

    This implements the camera model as defined in OpenCV.
    For details, see the OpenCV documentation.
    """
    def __init__(self, image_size, frame_rate, readout, camera_matrix, dist_coefs):
        """Create camera model

        Parameters
        -------------------
        image_size : tuple (rows, columns)
            The size of the image in pixels
        frame_rate : float
            The frame rate of the camera
        readout : float
            Rolling shutter readout time. Set to 0 for global shutter cameras.
        camera_matrix : (3, 3) ndarray
            The internal camera calibration matrix
        dist_coefs : ndarray
            Distortion coefficients [k1, k2, p1, p2 [,k3 [,k4, k5, k6]] of 4, 5, or 8 elements.
            Can be set to None to use zero parameters
        """
        super(OpenCVCameraModel, self).__init__(image_size, frame_rate, readout)
        self.camera_matrix = camera_matrix
        self.inv_camera_matrix = np.linalg.inv(self.camera_matrix)
        self.dist_coefs = dist_coefs

    def project(self, points):
        """Project 3D points to image coordinates.

        This projects 3D points expressed in the camera coordinate system to image points.

        Parameters
        --------------------
        points : (3, N) ndarray
            3D points

        Returns
        --------------------
        image_points : (2, N) ndarray
            The world points projected to the image plane
        """
        rvec = tvec = np.zeros(3)
        image_points, jac = cv2.projectPoints(points.T.reshape(-1,1,3), rvec, tvec, self.camera_matrix, self.dist_coefs)
        return image_points.reshape(-1,2).T

    def unproject(self, image_points):
        """Find (up to scale) 3D coordinate of an image point

        This is the inverse of the `project` function.
        The resulting 3D points are only valid up to an unknown scale.

        Parameters
        ----------------------
        image_points : (2, N) ndarray
            Image points

        Returns
        ----------------------
        points : (3, N) ndarray
            3D coordinates (valid up to scale)
        """
        undist_image_points = cv2.undistortPoints(image_points.T.reshape(1,-1,2), self.camera_matrix, self.dist_coefs, P=self.camera_matrix)
        world_points = np.dot(self.inv_camera_matrix, to_homogeneous(undist_image_points.reshape(-1,2).T))
        return world_points

def to_homogeneous(X):
    if X.ndim == 1:
        return np.append(X, 1)
    else:
        _, N = X.shape
        Y = np.ones((3, N))
        return np.vstack((X, np.ones((N, ))))

def from_homogeneous(X):
    Y = X / X[2]
    return Y[:2]