function pointCloud = DepthMapToPointCloud(depthMap, intrinsics)
% DepthMapToPointCloud produces a point cloud from depth map
%   pointCloud = DepthMapToPointCloud(depthMap, intrinsics) creates a point cloud from the distorted (taken directly from the sensor) depth image depthMap, after undistorting it
%                                                           via the intrinsics.
%
%   This function does no assertions. Make sure you provide it with legal inputs.

    depthMapUndistorted = Undistort(depthMap, intrinsics);
    
    height = size(depthMap, 1);
    width = size(depthMap, 2);
    
    pointCloud = zeros(height, width, 3);
    
    for v = 1:height
        for u = 1:width
            Z = depthMapUndistorted(v,u);
            X = (u - 1 - intrinsics.cx) * Z / intrinsics.fx; % -1 to compensate for matlab one based indexing
            X = X; % !!!! because Kinect output is right-left reflected
            Y = (v - 1 - intrinsics.cy) * Z / intrinsics.fy;
            pointCloud(v,u,:) = [X, Y, Z];
        end
    end
    
end

function undistorted = Undistort(distortedDepth, intrinsics)
% Undistort takes a distorted image and undistorts it according to intrinsics
%   undistored = Undistort(distortedDepth, intrinsics) undistorts distortedDepth with a (k1,k2,k3,p1,p2) based radial + tangential image distortion model. The 
%                                                 distortion coefficients reside in intrinsics. For additional info - check out https://en.wikipedia.org/wiki/Distortion_(optics) ).
%
%   This function does no assertions. Make sure you provide it with legal inputs.

    height = size(distortedDepth, 1);
    width = size(distortedDepth, 2);

    k1 = intrinsics.k1;
    k2 = intrinsics.k2;
    k3 = intrinsics.k3;
    p1 = intrinsics.p1;
    p2 = intrinsics.p2;
    cx = intrinsics.cx;
    cy = intrinsics.cy;
    fx = intrinsics.fx;
    fy = intrinsics.fy;
    
    undistorted = NaN(height, width);
    
    for v = 1:height
        for u = 1:width
            x_u = (u - 1 - cx) / fx; % -1 to compensate for matlab one based indexing
            y_u = (v - 1 - cy) / fy;
            r = norm([x_u, y_u]);
            radialCoefficient = (1 + k1*r^2 + k2*r^4 + k3*r^6);
            x_d = x_u * radialCoefficient + (r^2 + 2*x_u^2)*p2 + 2*x_u*y_u*p1;
            y_d = y_u * radialCoefficient + (r^2 + 2*y_u^2)*p1 + 2*x_u*y_u*p2;
            u_d = round(x_d * fx + cx) + 1; % +1 to compensate for matlab one based indexing 
            v_d = round(y_d * fy + cy) + 1;
            if ( u_d > 0 && u_d < width+1 && v_d > 0 && v_d < height+1) % i.e. - inside image 
                undistorted(v, u) = distortedDepth(v_d, u_d); 
            end
        end
    end

end

