%% bureaucracy
    
    visualize = true;
    numberOfImages = 100;
    numberOfCameras = 2;
    imageSourceDirectory = 'PlanarObject';
    imageSourceDirectories = {[imageSourceDirectory, '\0'], [imageSourceDirectory, '\1']};
    transformationFileName = 'Calibration\CalibrationResult.txt';
    
    roi = [ -1100 1100; -1100 1100; 0 1600];
    
    % take the intrinsics from libfreenect2: Freenect2Device::IrCameraParams
    intrinsics = { struct('fx', 365.52, 'fy', 365.52, 'cx', 256.919, 'cy', 207.111, 'k1', 0.0914316, 'k2', -0.274677, 'k3', 0.0974835, 'p1', 0, 'p2', 0),...
                   struct('fx', 365.306,'fy', 365.306,'cx', 254.963, 'cy', 208.328, 'k1', 0.0928791, 'k2', -0.270122, 'k3', 0.0916017, 'p1', 0, 'p2', 0) };
    % fx and fy (as all other distance measurements) in units of [mm]
    
%% load all point clouds

    pointClouds = cell(1, numberOfCameras);

    for i = 1:numberOfCameras
        pointClouds{i} = LoadPointClouds(imageSourceDirectories{i}, intrinsics{i}, numberOfImages);
    end
    
%% load and apply rigid transformation
    
    transformation = ReadExtrinsicCalibration(transformationFileName);        

    assert(numel(pointClouds{i}) == numel(pointClouds{2}));
    numberOfImages = numel(pointClouds{i});
    
    for i = 1:numberOfImages
        pointClouds{1}{i} = pctransform(pointClouds{1}{i}, transformation);
    end

%% Remove everything outside the ROI

    assert(all(size(roi) == [3 2]));

    for i = 1:2
        for j = 1:numel(pointClouds{i})
            disp(['Filtering: Camera ', num2str(i),', point cloud ', num2str(j)]);
            pointClouds{i}{j} = select(pointClouds{i}{j},findPointsInROI(pointClouds{i}{j}, roi));
        end
    end

%% Visualize point clouds in different color
    
    if visualize
        figure('units','normalized','outerposition',[0 0 1 1]);
        for i = 1:numberOfImages
            disp(['Displaying: point cloud ', num2str(i)]);
            pcshowpair(pointClouds{1}{i}, pointClouds{2}{i}, 'VerticalAxis', 'Y', 'VerticalAxisDir', 'Down');
            xlabel('X(mm)'); ylabel('Y(mm)'); zlabel('Z(mm)'); grid off;
            pause;
        end
    end
    
    
    