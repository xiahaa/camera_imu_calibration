function features = feature_detection(img, varargin)
    GFTT_PARAMS.max_corners = 100;
    GFTT_PARAMS.quality_level = 0.07;
    GFTT_PARAMS.min_distance = 10;
    if nargin >= 2
        if isempty(varargin{1})
            GFTT_PARAMS = varargin{1};
        end
    end
    
    if nargin >= 3
        use_mask = varargin{2};
    else
        use_mask = false;
    end
    
    if use_mask
        mask = ones(size(img));
        mask(end-200:end,:) = 0;
    end
    
    features = cv.goodFeaturesToTrack(img, 'MaxCorners',GFTT_PARAMS.max_corners, 'QualityLevel',GFTT_PARAMS.quality_level, 'MinDistance',GFTT_PARAMS.min_distance);
    if isempty(features)
        features = [];
        return;
    end 
end