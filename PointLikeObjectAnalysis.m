%% bureaucracy
    
    visualize = true;
    numberOfImages = 100;
    numberOfCameras = 2;
    imageSourceDirectories = {'PointLikeObject\0', 'PointLikeObject\1'};
    transformationFileName = 'Calibration\CalibrationResult.txt';
    
    roi = [ -1000 1000; -1000 1000; 0 1500];
    
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

%% Define the point-like object ROI and extract the associated points

roiCenters = { [ -114, -202, 1197], [-91, -232, 1112], [-291, -316, 1325], [-400 -357 1489] }; % , [-477, -237, 752], [2, -203, 1410] };    
roiSize = 100;

    for j = 1:numel(roiCenters)
        objectRoi{j} = [ roiCenters{j} - roiSize/2; roiCenters{j} + roiSize/2 ]';
        for i = 1:2
            objectRoiPointClouds{i}{j} = select(pointClouds{i}{j},findPointsInROI(pointClouds{i}{j}, objectRoi{j}));
        end
    end

%% visualize results

    if visualize
        for i = 1:numberOfImages
            figure('units','normalized','outerposition',[0 0 1 1]);
            disp(['Displaying: point cloud ', num2str(i)]);
            pcshowpair(pointClouds{1}{i}, pointClouds{2}{i}, 'VerticalAxis', 'Y', 'VerticalAxisDir', 'Down'); 
            hold on; pcshow(objectRoiPointClouds{1}{i}.Location, 'r', 'VerticalAxis', 'Y', 'VerticalAxisDir', 'Down'); 
            pcshow(objectRoiPointClouds{2}{i}.Location, 'b', 'VerticalAxis', 'Y', 'VerticalAxisDir', 'Down');
            xlabel('X(mm)'); ylabel('Y(mm)'); zlabel('Z(mm)'); grid off;
            pause;
            hold off;
        end
    end
    
    %% compute histograms

    figure;
    axes = {'X', 'Y', 'Z'};
    for i = 1:numberOfImages
        for j = 1:3
            subplot(numberOfImages, 3, (i-1)*3 + j);
            for k = 1:2
                histogram(objectRoiPointClouds{k}{i}.Location(:,j)/10, 'BinWidth', 0.5); hold on;
                title([axes{j}, '-axis, image ', num2str(i)]); xlabel('[cm]');
            end
            hold off;
        end
    end

%% compute center of mass and associated RMSE

    for i = 1:2
        centerOfMass{i} = cellfun(@(cloud) mean(cloud.Location) , objectRoiPointClouds{i}, 'UniformOutput', false);
    end
    
    Errors = cellfun(@(x,y) norm(x-y), centerOfMass{1}, centerOfMass{2})'
    RMSE = sqrt( mean( Errors.^2 ) )