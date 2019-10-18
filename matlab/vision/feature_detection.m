function features = feature_detection(img, varargin)
    GFTT_PARAMS.max_corners = 400;
    GFTT_PARAMS.quality_level = 0.07;
    GFTT_PARAMS.min_distance = 10;
    if nargin >= 2
        if ~isempty(varargin{1})
            GFTT_PARAMS = varargin{1};
        end
    end
    
    if nargin >= 3
        use_mask = varargin{2};
    else
        use_mask = false;
    end
    
    if nargin >= 4
        use_resize = varargin{3};
    else
        use_resize = true;
    end
    
    mask = uint8(ones(size(img)));
    if use_mask
        config;
        if isUAV==true
            % for test on M600
            if use_resize == true 
                mask(100:760,100:350)=0;
                mask(100:760,1050:1300)=0;
            else
                mask(200:1520,200:700)=0;
                mask(200:1520,2100:2600)=0;
            end
        else
            % for test on trolley
            mask(end-200:end,:) = 0;
        end
    end
    
    features = cv.goodFeaturesToTrack(img, 'MaxCorners',GFTT_PARAMS.max_corners, 'QualityLevel',GFTT_PARAMS.quality_level, ...
        'MinDistance',GFTT_PARAMS.min_distance,'Mask',mask);
    if isempty(features)
        features = [];
        return;
    end 
end