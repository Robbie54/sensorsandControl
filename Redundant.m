%% Assignment 1

%removing functions as they shouldn't be in main messes the workspace up
%need to make a bag class instead
clear all
close all
clc



% set(0,'DefaultFigureWindowStyle','docked')

%% Create Dobot

% % Displays the status
%     disp('Please wait creating dobot...');
%
%     % Creates the dobot
%     createDobot = Dobot();
%
%     hold on;
%     camlight;
%
%     % Displays the status
%     disp('Dobot Created yay, func end');
%     pause


%% Bag
%currently code just selects single message from camera topic and displays
%image
%there must be a better way

%I am unsure how to use the depth data for anything useful
% bag = rosbag('Modelnew2_360.bag');
%
%  rosbag info 'Modelnew2_360.bag';
% rosbag info 'CalibNew_360.bag';
% cameraInfo = select(bag, 'Topic','/camera/color/camera_info');
% cameraInfoMessage = readMessages(cameraInfo,50); %read topic (message 1)

%
% imageRaw = select(bag, 'Topic', '/camera/color/image_raw');
%
%
% message = readMessages(imageRaw,1); %read topic (message 1)
%
% %we need a better way then this to access the data and use it
%
% % imageData = message.Data;
% width = message{1}.Height; %get specific data from 1st message in topic
% height = message{1}.Width;
%
% imageData = message{1}.Data;
%
% allImageMessages = readMessages(imageRaw);
%
%
%
% R = imageData(1:3:end,:); %take one number every three from the data to get r data
% G = imageData(2:3:end,:);
% B = imageData(3:3:end,:);
%
% R = reshape(R, height,width); %reshape for imsho
% G = reshape(G, height,width);
% B = reshape(B, height,width);
%
%
% RGB = cat(3, R, G, B); %concatonate
% disp(' pause continuing will close figures and open image figure ');
%
% pause;
% close all
% hold on;
% imshow(RGB);

%% Rosbag Depth Reading
bag = rosbag('Modelnew2_360.bag');
depthTopic = select(bag, 'Topic', '/camera/depth/image_rect_raw');

depthTopicMessageNum = depthTopic.NumMessages;
depthImagesOut = readMessages(depthTopic);


%% Original Method
% % formatedDepthImages = zeros(depthTopicMessageNum,1);
%
% formatedDepthImages = cell(depthTopicMessageNum,10);
% for i = 1:depthTopicMessageNum
%     formatedDepthImages= readImage(depthImagesOut{i});
% end

% formatedDepthImage = readImage(depthImagesOut{1}); %some how readImage works was meant to be discontinued and now rosReadImage
% formatedDepthImage2 = readImage(depthImagesOut{2});
%
% imshow(uint8((double(formatedDepthImage)/350)*255)) %since this is a depth image we need to scale to be within uint8 range to display colour 350 was chosen based on largest number in data set


% % Initialize an empty cell array to store point clouds
% formatedDepthImages = cell(1, depthTopicMessageNum);
%
% for i = 1:depthTopicMessageNum
%     formatedDepthImages{i} = readImage(depthImagesOut{i});
%     imshow(uint8((double(formatedDepthImages{i}) / 350) * 255));
%     title(['Depth Image ' num2str(i)]);
%     % drawnow;  % Display the image immediately
%     % pause(0.1);  % Pause for a short duration to view each image
% end

%% Method Two - Creating a Combined Point Cloud
% % Initialize an empty cell array to store point clouds
% formatedDepthImages = cell(1, depthTopicMessageNum);
% pointClouds = cell(1, depthTopicMessageNum);
%
% % Initialize an empty point cloud to accumulate all point clouds
% masterPointCloud = pointCloud(zeros(0, 3));
%
% for i = 1:depthTopicMessageNum
%     formatedDepthImage = readImage(depthImagesOut{i});
%
%     % Calculate intrinsics for each depth image
%     imageSize = size(formatedDepthImage, [1, 2]);
%     focalLength = [610.339, 609.110]; % Adjust these values as needed
%     principalPoint = [317.109, 228.684]; % Adjust these values as needed
%     intrinsics = cameraIntrinsics(focalLength, principalPoint, imageSize);
%     depthScaleFactor = 5e3;
%
%     % Convert the depth image to a point cloud
%     pointCloud = pcfromdepth(formatedDepthImage, depthScaleFactor, intrinsics);
%
%     % Accumulate the current point cloud into the master point cloud
%     masterPointCloud = pcmerge(masterPointCloud, pointCloud, 0.01); % Adjust the threshold as needed
% end
%
% % Visualize the accumulated master point cloud
% figure;
% pcshow(masterPointCloud, 'VerticalAxis', 'Y', 'VerticalAxisDir', 'Up', 'ViewPlane', 'YX');
% title('Master Point Cloud');
% xlabel('X (m)');
% ylabel('Y (m)');
% zlabel('Z (m)');

%% Method Three - Creating a Combined Depth Image
% % Initialize a placeholder size for fullDepthMap
% fullDepthMap = [];
% 
% % Create an empty cell array to store intrinsics for each depth image
% intrinsicsList = cell(1, depthTopicMessageNum);
% 
% for i = 1:depthTopicMessageNum
%     depthImage = readImage(depthImagesOut{i});
% 
%     if isempty(fullDepthMap)
%         % If fullDepthMap is empty, set its size based on the first depth image
%         imageSize = size(depthImage, [1, 2]);
%         fullDepthMap = zeros(imageSize);
%     end
% 
%     % Convert depthImage to double
%     depthImage = double(depthImage);
% 
%     % Calculate intrinsics for the current depth image
%     imageSize = size(depthImage, [1, 2]);
%     focalLength = [610.339, 609.110]; % Adjust these values as needed
%     principalPoint = [317.109, 228.684]; % Adjust these values as needed
%     intrinsics = cameraIntrinsics(focalLength, principalPoint, imageSize);
%     depthScaleFactor = 5e3;
% 
%     % Store intrinsics in the list
%     intrinsicsList{i} = intrinsics;
% 
%     % Add the current depth image to the fullDepthMap
%     fullDepthMap = fullDepthMap + depthImage;
% end
%
% % Convert the stitched depth map into a 3D point cloud
% pointCloud = createPointCloud(fullDepthMap, intrinsicsList{1}, depthScaleFactor);
%
% % Visualize the 3D point cloud
% figure;
% pcshow(pointCloud, 'VerticalAxis', 'Y', 'VerticalAxisDir', 'Up', 'ViewPlane', 'YX');
% title('3D Environment Model');
% xlabel('X (m)');
% ylabel('Y (m)');
% zlabel('Z (m)');

%% Method Four - Seperate Point Clouds
% % Initialize an empty cell array to store point clouds
% formatedDepthImages = cell(1, depthTopicMessageNum);
% pointClouds = cell(1, depthTopicMessageNum);
%
% for i = 1:depthTopicMessageNum
%     formatedDepthImages{i} = readImage(depthImagesOut{i});
%
%     % Calculate intrinsics for the current depth image
%     imageSize = size(formatedDepthImages{i}, [1, 2]);
%     focalLength = [610.339, 609.110]; % Adjust these values as needed
%     principalPoint = [317.109, 228.684]; % Adjust these values as needed
%     intrinsics = cameraIntrinsics(focalLength, principalPoint, imageSize);
%     depthScaleFactor = 5e3;
%
%     % Convert the depth image to a point cloud
%     pointClouds{i} = pcfromdepth(formatedDepthImages{i}, depthScaleFactor, intrinsics);
%
%     % Display the point cloud
%     figure;
%     pcshow(pointClouds{i}, 'VerticalAxis', 'Y', 'VerticalAxisDir', 'Up', 'ViewPlane', 'YX');
%     title(['Point Cloud ' num2str(i)]);
%     xlabel('X (m)');
%     ylabel('Y (m)');
%     zlabel('Z (m)');
%
%     % drawnow;  % Display the point cloud immediately
%     % pause(0.1);  % Pause for a short duration to view each point cloud
% end

%% Method Five - Seperate Depth Clouds
% % Initialize an empty cell array to store point clouds
% formatedDepthImages = cell(1, depthTopicMessageNum);
%
% for i = 1:depthTopicMessageNum
%     formatedDepthImages{i} = readImage(depthImagesOut{i});
%     imshow(uint8((double(formatedDepthImages{i}) / 350) * 255));
%     title(['Depth Image ' num2str(i)]);
%     drawnow;  % Display the image immediately
%     pause(0.1);  % Pause for a short duration to view each image
% end

% Define camera parameters
focalLength = [610.339, 609.110]; % Adjust these values as needed
principalPoint = [317.109, 228.684]; % Adjust these values as needed
depthScaleFactor = 5e3;

% Define the indices of the selected point clouds
selectedIndices = [3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 19, 22, 23, 24, 26, 27, 28, 29, 30, 79, 80, 81, 82, 88, 89, 90, 93, 104, 106, 107, 108, 127, 131, 134, 135, 136, 137, 138, 140, 150, 152, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 190, 191];

% Initialize an empty cell array to store the selected point clouds
selectedPointClouds = cell(1, numel(selectedIndices));

% Define the camera intrinsics for all depth images (outside the loop)
intrinsicsList = cell(1, depthTopicMessageNum);
for k = 1:depthTopicMessageNum
    imageSize = size(depthImagesOut{k}, [1, 2]);
    intrinsicsList{k} = cameraIntrinsics(focalLength, principalPoint, imageSize);
end

% Iterate through the selected point clouds and store them in the cell array
for i = 1:numel(selectedIndices)
    index = selectedIndices(i);

    depthImage = readImage(depthImagesOut{index});

    % Calculate intrinsics for the current depth image using the correct index
    imageSize = size(depthImage, [1, 2]);
    intrinsics = intrinsicsList{index};

    % Convert the depth image to a 3D point cloud using the correct intrinsics
    pointCloud = createPointCloud(depthImage, intrinsics, depthScaleFactor);

    % Store the point cloud in the selectedPointClouds array
    selectedPointClouds{i} = pointCloud;
end

% Initialize the master point cloud with the first selected point cloud
masterPointCloud = selectedPointClouds{1};

% Merge the remaining selected point clouds one by one
for i = 2:numel(selectedIndices)
    masterPointCloud = pcmerge(masterPointCloud, selectedPointClouds{i}, 0.001); % You can adjust the mergeSize if needed
end

% Visualize the master point cloud
figure;
pcshow(masterPointCloud, 'VerticalAxis', 'Y', 'VerticalAxisDir', 'Up', 'ViewPlane', 'YX');
title('Selected Merged Point Cloud');
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');


% % Define the region of interest (ROI) limits
% roiXMin = -0.02;  % Specify the minimum X-coordinate of the ROI
% roiXMax = 0.07;  % Specify the maximum X-coordinate of the ROI
% roiYMin = -0.03;  % Specify the minimum Y-coordinate of the ROI
% roiYMax = 0.03;  % Specify the maximum Y-coordinate of the ROI
% roiZMin = 0.02;  % Specify the maximum Y-coordinate of the ROI
% roiZMax = 0.08;  % Specify the maximum Y-coordinate of the ROI
%
% % Create a binary mask to select points within the ROI
% roiMask = (masterPointCloud.Location(:, 1) >= roiXMin & masterPointCloud.Location(:, 1) <= roiXMax) ...
%     & (masterPointCloud.Location(:, 2) >= roiYMin & masterPointCloud.Location(:, 2) <= roiYMax) ...
%     & (masterPointCloud.Location(:, 2) >= roiZMin & masterPointCloud.Location(:, 2) <= roiZMax);
%
% % Apply the mask to select points within the ROI
% roiPointCloud = select(masterPointCloud, roiMask);
%
% % Visualize the filtered point cloud
% figure;
% pcshow(roiPointCloud, 'VerticalAxis', 'Y', 'VerticalAxisDir', 'Up', 'ViewPlane', 'YX');
% title('ROI Filtered Point Cloud');
% xlabel('X (m)');
% ylabel('Y (m)');
% zlabel('Z (m)');

% % Set axes limits
% xlim([-1, 1]); % Set the limits for the X-axis
% ylim([-1, 1]); % Set the limits for the Y-axis
% zlim([0, 2]); % Set the limits for the Z-axis



%% Original Point Cloud Attempt
% %% Camera Setup
%
% %these are now correct unless they change between messages
% %http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/CameraInfo.html
% focalLength      = [610.339, 609.110]; %taken from topic /camera/color/camera_info message 1
% principalPoint   = [317.109, 228.684];
%
% %NOT CORRECT intrinsics taken from help page need to find camera info
% imageSize        = size(formatedDepthImages{1},[1,2]);
% intrinsics       = cameraIntrinsics(focalLength,principalPoint,imageSize);
% depthScaleFactor = 5e3;
% maxCameraDepth   = 5;



% %% Point cloud
% ptCloud = pcfromdepth(formatedDepthImages{1}, depthScaleFactor, intrinsics);
% ptCloud2 = pcfromdepth(formatedDepthImages{2}, depthScaleFactor, intrinsics);
%
% pcshow(ptCloud, VerticalAxis="Y", VerticalAxisDir="Up", ViewPlane="YX");
%
% ptCloudRef = ptCloud;
% ptCloudCurrent = ptCloud2;
% gridSize = 0.1;
%
% %these might be wehre we put the em sensor trans and rot
% fixed = pcdownsample(ptCloudRef,gridAverage=gridSize);
% moving = pcdownsample(ptCloudCurrent,gridAverage=gridSize);
%
% tform = pcregistericp(ptCloud,ptCloud2,Metric="pointToPlane");
% ptCloudAligned = pctransform(ptCloudCurrent,tform);
%
% mergeSize = 0.015;
% ptCloudScene1 = pcmerge(ptCloudRef,ptCloudAligned,mergeSize);
%
% % Visualize the input images.
% figure
% subplot(2,2,1)
% imshow(uint8((double(formatedDepthImage)/350)*255))
% title("First input image",Color="w")
%
% subplot(2,2,3)
% imshow(uint8((double(formatedDepthImage2)/350)*255))
% title("Second input image",Color="w")
%
% % Visualize the world scene.
% subplot(2,2,[2,4])
% pcshow(ptCloudScene1,VerticalAxis="Y",VerticalAxisDir="Down")
% title("Initial world scene")
% xlabel("X (m)")
% ylabel("Y (m)")
% zlabel("Z (m)")
%
% ptCloudScene2 = helperStitchPointCloudsUsingColor(formatedDepthImages);
%
% figure
% pcshow(ptCloudScene2,VerticalAxis="Y",VerticalAxisDir="Down")
% title("Updated world scene with registration using color information")
% xlabel("X (m)")
% ylabel("Y (m)")
% zlabel("Z (m)")
%
%
%
%
%
% %% The below section is from the link on how to open a point cloud use for reference but not useful for bags
% %%From
% %%https://au.mathworks.com/help/vision/ug/3-d-point-cloud-registration-and-stitching.html?fbclid=IwAR3Amoezr36uL9f3yXKkBhPA5fCq7WxHiYWApa7ps96OtMsiiAO91k1QZe4
% fullfile(toolboxdir("vision"))
% dataFile = fullfile(toolboxdir("vision"),"visiondata","livingRoom.mat"); %this data is in a very different format to any one topic,
% %we need to work out what data from which topics we need
%
%
% load(dataFile);
%
% % Extract two consecutive point clouds and use the first point cloud as
% % reference.
% ptCloudRef = livingRoomData{1};
% ptCloudCurrent = livingRoomData{2};
% gridSize = 0.1;
% fixed = pcdownsample(ptCloudRef,gridAverage=gridSize);
% moving = pcdownsample(ptCloudCurrent,gridAverage=gridSize);
%
% tform = pcregistericp(moving,fixed,Metric="pointToPlane");
% ptCloudAligned = pctransform(ptCloudCurrent,tform);
%
% mergeSize = 0.015;
% ptCloudScene1 = pcmerge(ptCloudRef,ptCloudAligned,mergeSize);
%
% % Visualize the input images.
% figure
% subplot(2,2,1)
% imshow(ptCloudRef.Color)
% title("First input image",Color="w")
%
% subplot(2,2,3)
% imshow(ptCloudCurrent.Color)
% title("Second input image",Color="w")
%
% % Visualize the world scene.
% subplot(2,2,[2,4])
% pcshow(ptCloudScene1,VerticalAxis="Y",VerticalAxisDir="Down")
% title("Initial world scene")
% xlabel("X (m)")
% ylabel("Y (m)")
% zlabel("Z (m)")
%
%
% ptCloudScene2 = helperStitchPointCloudsUsingColor(livingRoomData);
%
% figure
% pcshow(ptCloudScene2,VerticalAxis="Y",VerticalAxisDir="Down")
% title("Updated world scene with registration using color information")
% xlabel("X (m)")
% ylabel("Y (m)")
% zlabel("Z (m)")
%
%
%
% %% Point Cloud Generation
% function ptCloudScene = helperStitchPointCloudsUsingColor(livingRoomData)
%
% % Extract the first point cloud as reference.
% ptCloudRef = livingRoomData{1};
%
% % Downsample the point cloud.
% gridSize = 0.1;
% moving = pcdownsample(ptCloudRef,gridAverage=gridSize);
%
% % Set the merge size to merge each point cloud to the scene.
% mergeSize = 0.015;
% ptCloudScene = ptCloudRef;
%
% % Store the transformation object that accumulates the transformation.
% accumTform = rigidtform3d();
%
% for i = 2:length(livingRoomData)
%     ptCloudCurrent = livingRoomData{i};
%
%     % Use previous moving point cloud as reference.
%     fixed = moving;
%     moving = pcdownsample(ptCloudCurrent,gridAverage=gridSize);
%
%     % Apply ICP registration.
%     tform = pcregistericp(moving,fixed,Metric="pointToPlaneWithColor",InlierDistance=0.1);
%
%     % Transform the current point cloud to the reference coordinate system
%     % defined by the first point cloud.
%     accumTform = rigidtform3d(accumTform.A * tform.A);
%     ptCloudAligned = pctransform(ptCloudCurrent,accumTform);
%
%     % Update the world scene.
%     ptCloudScene = pcmerge(ptCloudScene,ptCloudAligned,mergeSize);
% end
%
% % During the recording, the Kinect was pointing downward.
% % Transform the scene so that the ground plane is parallel
% % to the X-Z plane.
% angle = -10;
% translation = [0 0 0];
% tform = rigidtform3d([angle 0 0],translation);
% ptCloudScene = pctransform(ptCloudScene,tform);
% end
%

%% Create Point Cloud Function
function pointCloudFunc = createPointCloud(depthImage, intrinsics, depthScaleFactor)
[rows, cols] = size(depthImage);

% Initialize arrays to store X, Y, and Z coordinates
X = zeros(rows, cols);
Y = zeros(rows, cols);
Z = zeros(rows, cols);

% Apply intrinsics and depth scale factor to convert to 3D coordinates
for r = 1:rows
    for c = 1:cols
        Z(r, c) = double(depthImage(r, c)) / depthScaleFactor;
        X(r, c) = (c - intrinsics.PrincipalPoint(1)) * Z(r, c) / intrinsics.FocalLength(1);
        Y(r, c) = (r - intrinsics.PrincipalPoint(2)) * Z(r, c) / intrinsics.FocalLength(2);
    end
end

% Create a point cloud using the pointCloud constructor without color
XYZ = cat(3, X, Y, Z);
pointCloudFunc = pointCloud(XYZ);

% Visualize the point cloud (optional)
figure;
pcshow(pointCloudFunc);
title('3D Point Cloud');
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
end

%% Feature Detection Attempt
bag = rosbag('Modelnew2_360.bag');
depthTopic = select(bag, 'Topic', '/camera/depth/image_rect_raw');
depthTopicMessageNum = depthTopic.NumMessages;
depthImagesOut = readMessages(depthTopic);

for i = 1:depthTopicMessageNum
    original = readImage(depthImagesOut{1});
    distorted = readImage(depthImagesOut{i});

    ptsOriginal = detectSURFFeatures(original);
    ptsDistorted = detectSURFFeatures(distorted);

    [featuresOriginal, validPtsOriginal] = extractFeatures(original,ptsOriginal);
    [featuresDistorted, validPtsDistorted] = extractFeatures(distorted,ptsDistorted);

    indexPairs = matchFeatures(featuresOriginal,featuresDistorted);
    matchedOriginal = validPtsOriginal(indexPairs(:,1));
    matchedDistorted = validPtsDistorted(indexPairs(:,2));

    figure;
    showMatchedFeatures(original,distorted,matchedOriginal,matchedDistorted,'montage');

    [tform,inlierDistorted,inlierOriginal] = estimateGeometricTransform (matchedDistorted,matchedOriginal,'similarity');

    figure;
    showMatchedFeatures(original,distorted,inlierOriginal,inlierDistorted,'montage');
    title('Matching point (inliers only)');
    legend ('ptsOriginal','ptsDistorted');

    Tinv = tform.invert.T;
    ss = Tinv(2,1);
    sc = Tinv(1,1);
    scaleRecovered = sqrt(ss*ss+sc*sc);
    thetaRecovered = atan2(ss,sc)*180/pi;
    
    outputView = imref2d(size(original));
    recovered = imwarp(distorted,tform,'OutputView',outputView);
    figure;
    imshowpair(original,recovered,'montage');
end







