function pointClouds = LoadPointClouds( depthImagesDirectory, intrinsics, numberOfImages )
%LoadPointClouds returns a cell array of point clouds obtained from depthImagesDirectory and undistorted with intrinsics

    imageFiles = dir(fullfile(depthImagesDirectory, '*.png'));
    numberOfImages = min(numel(imageFiles), numberOfImages);
    disp(['detected ', num2str(numberOfImages), ' images in directory ', depthImagesDirectory]);       

    pointClouds = cell(1, numberOfImages);

    for i = 1:numberOfImages
       fileName = fullfile(depthImagesDirectory, imageFiles(i).name);
       disp(['processing ', fileName]);
       depth = imread(fileName);
       pointCloudMatrix = DepthMapToPointCloud(depth, intrinsics);       
       pointClouds{i} = pointCloud(pointCloudMatrix);
    end


end

