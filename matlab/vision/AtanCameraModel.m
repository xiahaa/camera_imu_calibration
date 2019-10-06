classdef AtanCameraModel < Camera
    properties
        camera_matrix;
        inv_camera_matrix;
        wc;
        lgamma;
    end
    
    methods
        function obj = AtanCameraModel(image_size, frame_rate, readout, ...
            camera_matrix, dist_center, dist_param)
            obj@Camera(image_size,frame_rate,readout);
            obj.camera_matrix = camera_matrix;
            obj.inv_camera_matrix = inv(obj.camera_matrix);
            obj.wc = dist_center;
            obj.lgamma = dist_param;
        end
        
        function unpoints = undistort(obj,points)
            wx = obj.wc(1);
            wy = obj.wc(2);
            % Switch to polar coordinates
            rn = sqrt((points(1,:) - wx).^2+(points(2,:) - wy).^2);
            phi = atan2(points(2,:) - wy,points(1,:) - wx);
            r = tan(rn * obj.lgamma) / obj.lgamma; % atan method
            unpoints = points;
            unpoints(1,:) = [wx + r.*cos(phi)];
            unpoints(2,:) = [wy + r.*sin(phi)];
        end
        
        function distortpoints = apply(obj,undisotpoints)
            wx = obj.wc(1);
            wy = obj.wc(2);
            rn = sqrt((undisotpoints(1,:) - wx).^2+(undisotpoints(2,:) - wy).^2);
            phi = atan2(undisotpoints(2,:) - wy,undisotpoints(1,:) - wx);
            r = atan(rn * obj.lgamma) / obj.lgamma;
            distortpoints = undisotpoints;
            distortpoints(1,:) = wx+r.*cos(phi);
            distortpoints(2,:) = wy+r.*sin(phi);
        end
        
        function imgpoints = project(obj,objpoints)
            objpoints = objpoints./objpoints(3,:);
            X = obj.apply(objpoints);
%             if size(X,1) == 2
%                 X = [X;ones(1,size(X,2))];
%             end
            x2d = obj.camera_matrix * X;
            imgpoints = x2d(1:2,:) ./ x2d(3,:);
        end

        function objpoints = unproject(obj,imgpoints)
            imgpoints = [imgpoints;ones(1,size(imgpoints,2))];
            objpoints = obj.inv_camera_matrix * imgpoints;
            objpoints = objpoints ./ objpoints(3,:);
            objpoints = obj.undistort(objpoints);
        end        
    end
end