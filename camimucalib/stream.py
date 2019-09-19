# -*- coding: utf-8 -*-
from __future__ import division, print_function, absolute_import

"""
Input streams module
"""
import os
import collections
import logging
logger = logging.getLogger('camera_imu_calibrator')

import cv2
import numpy as np

from . import tracking_new, rotations
from .imu_new import integrate_gyro_quaternion_uniform
from .imu_new import post_process_L3G4200D_data
from . import features

# Handle OpenCV 2.4.x -> 3.0
try:
    CV_CAP_PROP_POS_MSEC = cv2.cv.CV_CAP_PROP_POS_MSEC
except AttributeError:
    CV_CAP_PROP_POS_MSEC = cv2.CAP_PROP_POS_MSEC


class GyroStream(object):
    def __init__(self):
        self.__last_dt = None
        self.__last_q = None
        self.data = None # Arranged as Nx3 where N is number of samples

    @classmethod
    def from_csv(cls, filename):
        """Create gyro stream from CSV data

        Load data from a CSV file.
        The data must be formatted with three values per line: (x, y, z)
        where x, y, z is the measured angular velocity (in radians) of the specified axis.

        Parameters
        -------------------
        filename : str
            Path to the CSV file

        Returns
        ---------------------
        GyroStream
            A gyroscope stream
        """
        instance = cls()
        instance.data = np.loadtxt(filename, delimiter=',')
        return instance

    @classmethod
    def from_data(cls, data):
        """Create gyroscope stream from data array

        Parameters
        -------------------
        data : (N, 3) ndarray
            Data array of angular velocities (rad/s)

        Returns
        -------------------
        GyroStream
            Stream object
        """
        if not data.shape[1] == 3:
            raise ValueError("Gyroscope data must have shape (N, 3)")

        instance = cls()
        instance.data = data
        return instance

    @property
    def num_samples(self):
        return self.data.shape[0]

    def prefilter(self, do_plot = False):
        if not self.data.any() == None:
            self.data = post_process_L3G4200D_data(self.data.T, do_plot).T
        else:
            raise Exception("Load first and then do prefiltering")
        return

    def integrate(self, dt):
        """Integrate gyro measurements to orientation using a uniform sample rate.

        Parameters
        -------------------
        dt : float
            Sample distance in seconds

        Returns
        ----------------
        orientation : (4, N) ndarray
                    Gyroscope orientation in quaternion form (s, q1, q2, q3)
        """
        if not dt == self.__last_dt:
            self.__last_q = integrate_gyro_quaternion_uniform(self.data, dt)
            self.__last_dt = dt
        return self.__last_q

class VideoStream(object):
    """Video stream representation

    This is the base class for all video streams, and should normally not be used directly.
    Instead you should use a VideoStream subclass that can work on the data you have.

    The concept of a video stream can be summarized as something that
    "provides frames of video data captured by a single camera".

    A VideoStream object is iterable to allow reading frames easily::

        stream = VideoStreamSubClass(SOME_PARAMETER)
        for frame in stream:
            do_stuff(frame)

    """
    def __init__(self, camera_model, flow_mode='optical'):
        """Create a VideoStream object

        Parameters
        ----------------
        camera_model : CameraModel
                     Camera model used by this stream
        """
        self._flow = None
        self.flow_mode = flow_mode
        self.camera_model = camera_model

    def __iter__(self):
        return self._frames()

    def _frames(self):
        raise NotImplementedError("{} does not implement the _frames() method used to extract frames".format(self.__class__.__name__))

    @classmethod
    def from_file(cls, camera_model, filename):
        """Create stream automatically from filename.

        Note
        --------------------
        This currently only works with video files that are readable by OpenCV

        Parameters
        --------------------
        camera_model : CameraModel
            Camera model to use with this stream
        filename : str
            The filename to load the stream data from

        Returns
        --------------------
        VideoStream
            Video stream of a suitable sub class
        """
        # TODO: Other subclasses
        return OpenCvVideoStream(camera_model, filename)

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
            The world points projected to the image plane of the camera used by the stream
        """
        return self.camera_model.project(points)

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
        return self.camera_model.unproject(image_points)

    def display_video(self):
        cv2.namedWindow('video', 0)
        cv2.resizeWindow('video',640,480)
        for frame in self:
            frame_show=cv2.imresize(frame,640,480)
            cv2.imshow('video',frame_show)
            cv2.waitKey(1)
        cv2.destroyWindow('video')
        return

    @property
    def frame_rate(self):
        return self.camera_model.frame_rate

    @property
    def flow(self):
        if self._flow is None:
            logger.debug("Generating flow. This can take minutes depending on video length")
            if self.flow_mode == 'rotation':
                self._generate_frame_to_frame_rotation()
            elif self.flow_mode == 'optical':
                self._flow = tracking_new.frametoframe_track(self)
            else:
                raise ValueError("No such flow mode '{}'".format(self.flow_mode))
            #self.__generate_flow()
        return self._flow

    def _generate_frame_to_frame_rotation(self):
        rotation = []
        weights = []
        step = 1
        maxlen = step + 1

        gftt_params = {
            'max_corners': 500,
            'quality_level': 0.07,
            'min_distance': 10,
        }

        frame_queue = collections.deque([], maxlen)
        for frame in self:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            frame_queue.append(frame)
            if len(frame_queue) == maxlen:
                initial_pts = features.feature_detection(frame_queue[0],gftt_params)
                pts, status = tracking_new.track_retrack(list(frame_queue), initial_pts)
                X = pts[:, 0, :].T
                Y = pts[:, 1, :].T
                threshold = 2.0
                R, _, err, inliers = rotations.estimate_rotation_procrustes_ransac(X, Y, self.camera_model, threshold)
                if R is None:
                    weight = 0
                    angle = 0
                    r = np.zeros(3)
                else:
                    weight = (1.0 * len(inliers)) / len(pts)
                    axis, angle = rotations.rotation_matrix_to_axis_angle(R)
                    r = axis * angle
                rotation.append(r.reshape(3, 1))
                weights.append(weight)

        rotation = np.hstack(rotation)
        weights = np.array(weights)

        # Scale from rad/frame to rad/s
        rotation *= self.camera_model.frame_rate

        # Remove and interpolate bad values
        threshold = 0.2
        mask = weights > threshold
        x = np.arange(rotation.shape[1])
        for i in range(3):
            rotation[i, ~mask] = np.interp(x[~mask], x[mask], rotation[i, mask])

        self._frame_rotations = rotation
        self._flow = np.linalg.norm(rotation, axis=0)

class OpenCvVideoStream(VideoStream):
    """Video stream that uses OpenCV to extract image data.

    This stream class uses the OpenCV VideoCapture class and can thus handle any
    video type that is supported by the installed version of OpenCV.
    It can only handle video files, and not live streams.
    """
    def __init__(self, camera_model, filename, start_time=0.0, duration=None):
        """Create video stream

        Parameters
        ---------------
        camera_model : CameraModel
            Camera model
        filename : str
            Path to the video file
        start_time : float
            The time in seconds where to start capturing (USE WITH CAUTION)
        duration : float
            Duration in seconds to capture (USE WITH CAUTION)

        Notes
        -------------------
        You can specify the start time and duration you want to use for the capture.
        However, be advised that this may or may not work depending on the type of video data
        and your installation of OpenCV. Use with caution!
        """
        super(OpenCvVideoStream, self).__init__(camera_model)
        self.filename = filename
        self.start_time = start_time
        self.duration = duration
        self.step = 1

    def _frames(self):
        vc = cv2.VideoCapture(self.filename)
        if not vc.isOpened():
            raise IOError("Failed to open '{}'. Either there is something wrong with the file or OpenCV does not have the correct codec".format(self.filename))
        # OpenCV does something really stupid: to set the frame we need to set it twice and query in between
        t = self.start_time * 1000. # turn to milliseconds
        t2 = t + self.duration*1000.0 if self.duration is not None else None

        for i in range(2): # Sometimes needed for setting to stick
            vc.set(CV_CAP_PROP_POS_MSEC, t)
            vc.read()
        t = vc.get(CV_CAP_PROP_POS_MSEC)
        counter = 0
        retval = True
        while retval and (t2 is None or (t2 is not None and t < t2)):
            retval, im = vc.read()
            if retval:
                if np.mod(counter, self.step) == 0:
                    yield im
            elif t2 is not None:
                raise IOError("Failed to get frame at time %.2f" % t)
            else:
                pass # Loop will end normally
            t = vc.get(CV_CAP_PROP_POS_MSEC)
            counter += 1
