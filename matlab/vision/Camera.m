classdef Camera < handle
    properties
        image_size;
        frame_rate;
        readout;
        rows;
        cols;
    end
    methods 
        function obj = Camera(imagesize, framerate, readout)
            obj.image_size = imagesize;
            obj.frame_rate = framerate;
            obj.readout = readout;
            obj.rows = imagesize(2);
            obj.cols = imagesize(1);
        end
        
        function imgpts = project(obj,objpts)
            % 
            error('sub-class should implemnt this methods!');
        end
        
        function objpts = unproject(obj,imgpts)
            error('sub-class should implemnt this methods!');
        end
    end
end