from __future__ import division, print_function, absolute_import

import time
import warnings
import numpy as np 
import matplotlib.pyplot as plt
import scipy.optimize
import logging 
logger = logging.getLogger('calib_xiahaa')

from . import rotations, tracking, videoslice

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
    
    def initialize(self, gyro_rate, slices=None, skip_estimation=False):
        self.params['user']['gyro_rate'] = gyro_rate
        logger.info('gyro_rate is provided!')

        for p in ('gbias_x', 'gbias_y', 'gbias_z'):
            self.params['initialized'][p] = 0.0
        logger.info('gyro initalized bias: {:f, :f, :f}'.format(self.params['initialized']['gbias_x'],self.params['initialized']['gbias_y'],self.params['initialized']['gbias_z']))

        if slices is not None:
            self.slices = slices

        if self.slices is None:
            self.slices = videoslice.Slice.from_stream_randomly(self.video)
            logger.debug("Number of slices: {:d}".format(len(self.slices)))

        if len(self.slices) < 2:
            logger.error("Calibration requires at least 2 video slices to proceed, got %d", len(self.slices))
            raise Exception("Calibration requires at least 2 video slices to proceed, got {:d}".format(len(self.slices)))

        if not skip_estimation:
            time_offset = self.find_initial_offset()
            # TODO: Detect when time offset initialization fails, and raise InitializationError

            R = self.find_initial_rotation()
            if R is None:
                raise Exception("Failed to calculate initial rotation")

    def find_initial_offset():
        