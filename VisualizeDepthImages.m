targetDirectory = 'D:\repositories\Kinect2ClientSide\Data';

firstCameraTargetDir = fullfile(targetDirectory,'0');
secondCameraTargetDir = fullfile(targetDirectory,'1');

assert(exist(firstCameraTargetDir,'dir') && exist(secondCameraTargetDir,'dir'));

firstDepthImages = dir(fullfile(firstCameraTargetDir, '*.png'));
secondDepthImages = dir(fullfile(secondCameraTargetDir, '*.png'));

assert(numel(firstDepthImages) == numel(secondDepthImages))

numberOfImages = numel(firstDepthImages);

figure;
for i = 1:numberOfImages
    subplot(1,2,1);
    M1 = imread(fullfile(firstCameraTargetDir,firstDepthImages(i).name)); 
    imagesc(M1); 
    subplot(1,2,2);
    M2 = imread(fullfile(secondCameraTargetDir,secondDepthImages(i).name)); 
    imagesc(M2);  
    disp(i);
    pause;
end