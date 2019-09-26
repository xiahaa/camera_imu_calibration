function flow = flow_from_video(videofile, start_time, duration)
    % setup video capture
    cap = cv.VideoCapture(videofile);
    assert(cap.isOpened());
    frame = cap.read();
    assert(~isempty(frame) && size(frame,3)==3);
    prev = cv.cvtColor(frame, 'RGB2GRAY');
    
end