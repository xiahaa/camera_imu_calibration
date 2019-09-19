from __future__ import division, print_function, absolute_import

import cv2
import numpy as np 
import logging

logger = logging.getLogger('features')

GFTT_PARAMS = {
    'max_corners': 100,
    'quality_level': 0.07,
    'min_distance': 10,
}

def feature_detection(img, gftt_options=[]):
    # initialize gftt parameters
    if gftt_options:
        gftt_params = gftt_options
    else:
        gftt_params = GFTT_PARAMS
    features = cv2.goodFeaturesToTrack(img,gftt_params['max_corners'], \
        gftt_params['quality_level'],gftt_params['min_distance'])
    return features