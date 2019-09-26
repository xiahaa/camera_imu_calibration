function refined_offset=refine_time_offset(images, frame_timestamps, ...
    rotation_sequence, rotation_timestamps, camera_matrix, readout_time)
% """Refine a time offset between camera and IMU using rolling shutter aware optimization.
% 
%     To refine the time offset using this function, you must meet the following constraints
% 
%     1) The data must already be roughly aligned. Only a few image frames of error
%         is allowed.
%     2) The images *must* have been captured by a *rolling shutter* camera.
% 
%     This function finds a refined offset using optimization.
%     Points are first tracked from the start to the end of the provided images.
%     Then an optimization function looks at the reprojection error of the tracked points
%     given the IMU-data and the refined offset.
% 
%     The found offset *d* is such that you want to perform the following time update
% 
%         new_frame_timestamps = frame_timestamps + d
% 
%     Parameters
%     ------------
%     image_list : list of ndarray
%             A list of images to perform tracking on. High quality tracks are required,
%             so make sure the sequence you choose is easy to track in.
%     frame_timestamps : ndarray
%             Timestamps of image_list
%     rotation_sequence : (4, N) ndarray
%             Absolute rotations as a sequence of unit quaternions (first element is scalar).
%     rotation_timestamps : ndarray
%             Timestamps of rotation_sequence
%     camera_matrix : (3,3) ndarray
%             The internal camera calibration matrix of the camera.
%     readout_time : float
%             The readout time of the camera.
% 
%     Returns
%     ------------
%     offset : float
%             A refined offset that aligns the image data with the rotation data.
%     """
    error('not yet implemented!');
    % todo
end


%     
%     # ) Track points
%     max_corners = 200
%     quality_level = 0.07
%     min_distance = 5
%     max_tracks = 20
%     initial_points = cv2.goodFeaturesToTrack(image_list[0], max_corners, quality_level, min_distance)
%     (points, status) = tracking.track_retrack(image_list, initial_points)
% 
%     # Prune to at most max_tracks number of tracks, choose randomly
%     track_id_list = np.random.permutation(points.shape[0])[:max_tracks]
% 
%     rows, cols = image_list[0].shape[:2]
%     row_delta_time = readout_time / rows
%     num_tracks, num_frames, _ = points.shape
%     K = np.matrix(camera_matrix)
% 
%     def func_to_optimize(td, *args):
%         res = 0.0
%         N = 0
%         for frame_idx in range(num_frames-1):
%             for track_id in track_id_list:
%                 p1 = points[track_id, frame_idx, :].reshape((-1,1))
%                 p2 = points[track_id, frame_idx + 1, :].reshape((-1,1))
%                 t1 = frame_timestamps[frame_idx] + (p1[1] - 1) * row_delta_time + td
%                 t2 = frame_timestamps[frame_idx + 1] + (p2[1] - 1) * row_delta_time +td
%                 t1 = float(t1)
%                 t2 = float(t2)
%                 q1 = IMU.rotation_at_time(t1, rotation_timestamps, rotation_sequence)
%                 q2 = IMU.rotation_at_time(t2, rotation_timestamps, rotation_sequence)
%                 R1 = rotations.quat_to_rotation_matrix(q1)
%                 R2 = rotations.quat_to_rotation_matrix(q2)
%                 p1_rec = K.dot(R1.T).dot(R2).dot(K.I).dot(np.vstack((p2, 1)))
%                 if p1_rec[2] == 0:
%                     continue
%                 else:
%                     p1_rec /= p1_rec[2]
%                 res += np.sum((p1 - np.array(p1_rec[0:2]))**2)
%                 N += 1
%         return res / N
% 
%     # Bounded Brent optimizer
%     t0 = time.time()
%     tolerance = 1e-4 # one tenth millisecond
%     (refined_offset, fval, ierr, numfunc) = scipy.optimize.fminbound(func_to_optimize, -0.12, 0.12, xtol=tolerance, full_output=True)
%     t1 = time.time()
%     if ierr == 0:
%         logger.info("Time offset found by brent optimizer: %.4f. Elapsed: %.2f seconds (%d function calls)", refined_offset, t1-t0, numfunc)
%     else:
%         logger.error("Brent optimizer did not converge. Aborting!")
%         raise Exception("Brent optimizer did not converge, when trying to refine offset.")
% 
%     return 