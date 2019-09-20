#%%
#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function, absolute_import
"""
This is an example script that shows how to run the calibrator on our dataset.
The dataset can be found here:

    http://www.cvl.isy.liu.se/research/datasets/gopro-gyro-dataset/

To run, simply point the script to one of the video files in the directory

    $ python gopro_gyro_dataset_example.py /path/to/dataset/video.MP4
"""

import os
import sys
import argparse

import numpy as np

sys.path.insert(0,'./')

import camimucalib
from camimucalib.calibration_new import PARAM_ORDER

CAMERA_MATRIX = np.array(
    [[ 853.12703455,    0.        ,  988.06311256],
     [   0.        ,  873.54956631,  525.71056312],
     [   0.        ,    0.        ,    1.        ]]
)
CAMERA_DIST_CENTER = (0.00291108,  0.00041897)
CAMERA_DIST_PARAM = 0.8894355
CAMERA_FRAME_RATE = 30.0
CAMERA_IMAGE_SIZE = (1920, 1080)
CAMERA_READOUT = 0.0316734
GYRO_RATE_GUESS = 853.86

#%%
parser = argparse.ArgumentParser()
parser.add_argument('video')
args = parser.parse_args(['./data/gopro-gyro-dataset/walk.MP4'])
print(args.video)
gyro_file = os.path.splitext(args.video)[0] + '_gyro.csv'
reference_file = os.path.splitext(args.video)[0] + '_reference.csv'
print(gyro_file)

#%%
print('Creating gyro stream from {}'.format(gyro_file))
gyro = camimucalib.GyroStream.from_csv(gyro_file)
print('Post processing L3G4200D gyroscope data to remove frequency spike noise')
gyro.prefilter(do_plot=False)

#%%
camera = camimucalib.AtanCameraModel(CAMERA_IMAGE_SIZE, CAMERA_FRAME_RATE, CAMERA_READOUT, CAMERA_MATRIX,
                                   CAMERA_DIST_CENTER, CAMERA_DIST_PARAM)
print('Creating video stream from {}'.format(args.video))
video = camimucalib.VideoStream.from_file(camera, args.video)
#video.display_video()
#%%
calib = camimucalib.calibration_new.calibrator(video,gyro)
calib.initialize(gyro_rate=GYRO_RATE_GUESS,is_debug=True)

#%%
#import matplotlib.pyplot as plt
#plt.plot(calib.flow)

#%%
