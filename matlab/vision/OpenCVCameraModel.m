classdef OpenCVCameraModel < Camera
    properties
        camera_matrix;
        inv_camera_matrix;
        dist_coefs;
    end
    
    methods
        function obj = OpenCVCameraModel(image_size, frame_rate, readout, camera_matrix, dist_coefs)
            obj@Camera(image_size, frame_rate, readout);
            obj.camera_matrix=camera_matrix;
            obj.inv_camera_matrix = inv(camera_matrix);
            obj.dist_coefs = dist_coefs;
        end
        
        function imagePoints = project(obj,points)
            imagePoints = cv.projectPoints(points, [0,0,0], [0,0,0], obj.camera_matrix, obj.dist_coefs);
        end
        
        function points1 = undistort(obj,points)
            points1 = cv.undistortPoints(points,obj.camera_matrix, obj.dist_coefs);
        end

        function objpoints = unproject(obj,image_points)
            undist_image_points = cv.undistortPoints(image_points, ...
                obj.camera_matrix, obj.dist_coefs, 'P', obj.camera_matrix);
            undist_image_points = [undist_image_points;ones(1,size(undist_image_points,2))];
            objpoints = obj.inv_camera_matrix * undist_image_points;
        end
    end
end