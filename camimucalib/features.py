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

# other features can be add in the future

def feature_detection(img, gftt_options=[], use_mask = True):
    # initialize gftt parameters
    if gftt_options:
        gftt_params = gftt_options
    else:
        gftt_params = GFTT_PARAMS

    if use_mask:
        mask = np.zeros_like(img)
        rows = img.shape[0]
        cols = img.shape[1]
        # mask out bottom 200 lines
        mask[:-200] = 1

    features = cv2.goodFeaturesToTrack(img,gftt_params['max_corners'], \
        gftt_params['quality_level'],gftt_params['min_distance'],mask=mask)
    return features