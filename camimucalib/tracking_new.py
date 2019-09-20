from __future__ import division, print_function, absolute_import

import cv2
import numpy as np 
import logging
from matplotlib import pyplot as plt
from . import features
logger = logging.getLogger('tracking')

def frametoframe_track(imgseq, max_diff=60, gftt_options=[],do_plot=False):
    flow=[]
    prev_img = None

    for img in imgseq:
        if img.ndim == 3 and img.shape[2] == 3:
            # to gray-scale
            img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if prev_img is None:
            prev_img = img
            continue
        # detect features in previous img
        prev_points = features.feature_detection(prev_img)

        # use optical flow tracking
        [_points, _status, _err] = cv2.calcOpticalFlowPyrLK(prev_img, img, \
            prev_points,np.array([]))
        # NxMx2, here M=1
        # print(_points.shape)
        
        # simple filter
        valids = np.nonzero(_status)
        new_points = _points[valids]
        prev_points = prev_points[valids]
        # compute 
        distance = np.sqrt(np.sum((new_points-prev_points)**2,1))
        valids = distance < max_diff
        # return flow strength
        dm = np.mean(distance[valids])
        if np.isnan(dm):
            dm = 0
        flow.append(dm)
        prev_img = img

        if do_plot == True:
            cv2.namedWindow('frame to frame tracking',cv2.WINDOW_KEEPRATIO)
            imgc = cv2.cvtColor(img,cv2.COLOR_GRAY2BGR)
            corners1 = np.int0(new_points)
            corners2 = np.int0(prev_points)
            for i,j in zip(corners1,corners2):
                x2,y2 = i.ravel()
                x1,y1 = j.ravel()
                cv2.circle(imgc,(x2,y2),3,(0,0,255),-1)
                cv2.circle(imgc,(x1,y1),3,(0,255,255),-1)
                cv2.line(imgc,(x1,y1),(x2,y2),(255,255,255),1)
            imgc = cv2.resize(imgc,(640,480))
            #plt.imshow(imgc), plt.show()
            cv2.imshow('frame to frame tracking',imgc)
            cv2.waitKey(1)
    # return flow strength
    if do_plot == True:
       cv2.destroyWindow('frame to frame tracking')
    return np.array(flow)

def track(imgseq, initial_points, remove_bad = False,do_plot=False):
    # NxMx2
    tracks = np.zeros((initial_points.shape[0],len(imgseq),2),dtype=np.float32)
    # for broadcast
    tracks[:,0,:] = np.reshape(np.array(initial_points),[-1,2])
    track_status = np.ones([np.size(initial_points,0),1])
    placeholder = np.array([])
    window_size = (5,5)
    for i in range(1,len(imgseq)):
        img1 = imgseq[i-1]
        img2 = imgseq[i]
        # equivalent to track_status != 1, return is the index
        prev_ok_status = np.flatnonzero(track_status)
        # get previsou ok feature points
        prev_points = tracks[prev_ok_status,i-1,:]
        [points, status, err]=cv2.calcOpticalFlowPyrLK(img1,img2,prev_points,\
            placeholder,placeholder,placeholder,window_size)
        if status is None:
            # all are lost
            track_status[:] = 0
            break
        # valid index
        valids = np.flatnonzero(status)
        now_ok_status = prev_ok_status[valids]
        tracks[now_ok_status,i,:] = points[valids]
        track_status[prev_ok_status] = status

        if do_plot == True:
            cv2.namedWindow('slice tracking',cv2.WINDOW_KEEPRATIO)
            imgc = cv2.cvtColor(img2,cv2.COLOR_GRAY2BGR)
            corners1 = np.int0(tracks[now_ok_status,0,:])
            corners2 = np.int0(tracks[now_ok_status,i,:])
            for i,j in zip(corners1,corners2):
                x2,y2 = i.ravel()
                x1,y1 = j.ravel()
                cv2.circle(imgc,(x2,y2),3,(0,0,255),-1)
                cv2.circle(imgc,(x1,y1),3,(0,255,255),-1)
                cv2.line(imgc,(x1,y1),(x2,y2),(255,255,255),1)
            imgc = cv2.resize(imgc,(640,480))
            #plt.imshow(imgc), plt.show()
            cv2.imshow('slice tracking',imgc)
            cv2.waitKey(1)
    # return flow strength
    if do_plot == True:
       cv2.destroyWindow('slice tracking')

    if remove_bad:
        final_ok = np.flatnonzero(track_status)
        tracks = tracks[final_ok]
        track_status = track_status[final_ok]
    else:
        pass
    return (tracks,track_status)

def track_retrack(imgseq,initial_points,max_retrack_distance=0.5,keep_bad=False,do_plot=False):
    (forward_track, forward_status) = track(imgseq,initial_points,False,do_plot)
    (backward_track, backward_status) = track(imgseq[::-1],forward_track[:,-1,:],False,do_plot)
    
    ok_status = np.flatnonzero(forward_status * backward_status)
    # features in the first frame
    points1 = forward_track[ok_status,0,:]
    # retracked features in the first frame (last in the backward mode)
    points2 = backward_track[ok_status,-1,:]
    # deviation
    retrack_distance = np.sqrt(np.sum((points1-points2)**2,1))
    # good track
    valids = np.flatnonzero(retrack_distance < max_retrack_distance)
    final_status = ok_status[valids]

    if do_plot == True:
        cv2.namedWindow('slice tracking',cv2.WINDOW_KEEPRATIO)
        imgc = cv2.cvtColor(imgseq[-1],cv2.COLOR_GRAY2BGR)
        corners1 = np.int0(forward_track[final_status,0,:])
        corners2 = np.int0(forward_track[final_status,-1,:])
        for i,j in zip(corners1,corners2):
            x2,y2 = i.ravel()
            x1,y1 = j.ravel()
            cv2.circle(imgc,(x2,y2),5,(0,0,255),-1)
            cv2.circle(imgc,(x1,y1),5,(0,255,255),-1)
            cv2.line(imgc,(x1,y1),(x2,y2),(0,255,0),1)
        imgc = cv2.resize(imgc,(640,480))
        #plt.imshow(imgc), plt.show()
        cv2.imshow('slice tracking',imgc)
        cv2.waitKey(0)
    # return flow strength
    if do_plot == True:
        cv2.destroyWindow('slice tracking')
    # 
    if not keep_bad:
        return (forward_track[final_status], forward_status[final_status])
    else:
        forward_status = np.zeros(np.size(forward_status,0),1)
        forward_status[final_status] = 1
        return (forward_track, forward_status)



