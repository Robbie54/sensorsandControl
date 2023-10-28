%% File Setup
clear all
clf
close all
clc

%% Rosbag Depth Reading
bag = rosbag('Modelnew2_360.bag');
depthTopic = select(bag, 'Topic', '/camera/depth/image_rect_raw');
allDepthImages = readMessages(depthTopic);
start_index = 51;
end_index = 100;
if end_index > depthTopic.NumMessages
    end_index = depthTopic.NumMessages;
end

%% Define camera parameters
%focalLength = [610.339, 609.110]; % Adjust these values as needed
%principalPoint = [317.109, 228.684]; % Adjust these values as needed
depthScaleFactor = 5e3;

%% Depth camera intrinsics extracted from ModelNewBag topic /camera/depth/camera_info
K = [421.7674560546875, 0, 423.2069396972656, 0, 421.7674560546875, 239.1299591064453, 0, 0, 1];
focalLength = K(1);
principalPoint = [K(3), K(6)]; %x and y respectively

%% Iterate through depth images 51-100 and create a point cloud
pointClouds = cell(1, end_index - start_index + 1);
roi = [-0.1 0.1 -0.05 0.1 0 0.2];
for k = start_index:end_index
    depthImage = readImage(allDepthImages{k});

    % Calculate intrinsics for the current depth image
    imageSize = size(depthImage, [1, 2]);
    intrinsics = cameraIntrinsics(focalLength, principalPoint, imageSize);

    % Convert the depth image to a 3D point cloud
    pointCloud = createPointCloud(depthImage, intrinsics, depthScaleFactor);

    %Apply ROI filter to the current point cloud
    indices = findPointsInROI(pointCloud, roi);
    roiPointCloud = select(pointCloud,indices);

    % Store the point cloud in the cell array
    pointClouds{k - start_index + 1} = roiPointCloud;
end

% Visualize all the point clouds
for k = 1:length(pointClouds)
    figure;
    pcshow(pointClouds{k}, 'VerticalAxis', 'Y', 'VerticalAxisDir', 'Down');
    title(['Point Cloud ', num2str(k + start_index - 1)]);
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
end


%%main code 

% pcshow(pointClouds{1}, 'VerticalAxis', 'Y', 'VerticalAxisDir', 'Down');
%     title(['Point Cloud ', num2str(k + start_index - 1)]);
%     xlabel('X (m)');
%     ylabel('Y (m)');
%     zlabel('Z (m)');
% 
% 
% 
%     roi = [-0.1 0.1 -0.05 0.1 0 0.2];
%     indices = findPointsInROI(pointClouds{1},roi);
% 
% 
%     roiPointCloud = select(pointClouds{1}, indices);

 
% hopeful = removeOutliers(pointClouds{1});
% pcshow(hopeful, 'VerticalAxis', 'Y', 'VerticalAxisDir', 'Down');
%     title(['new Cloud ', num2str(k + start_index - 1)]);
%     xlabel('X (m)');
%     ylabel('Y (m)');
%     zlabel('Z (m)');


% 
% %% File Setup
% clear all
% clf
% close all
% clc
% 
% %% Rosbag Depth Reading
% bag = rosbag('Modelnew2_360.bag');
% depthTopic = select(bag, 'Topic', '/camera/depth/image_rect_raw');
% depthTopicMessageNum = depthTopic.NumMessages;
% depthImagesOut = readMessages(depthTopic);
% 
% %% Define camera parameters
% %focalLength = [610.339, 609.110]; % Adjust these values as needed
% %principalPoint = [317.109, 228.684]; % Adjust these values as needed
% depthScaleFactor = 5e3;
% 
% %% Depth camera intrinsics extracted from ModelNewBag topic /camera/depth/camera_info
% K = [421.7674560546875, 0, 423.2069396972656, 0, 421.7674560546875, 239.1299591064453, 0, 0, 1];
% focalLength = K(1);
% principalPoint = [K(3),K(6)]; %x and y respectively 
% 
% 
% 
% 
% 
% 
% 
% %% Define the indices of the selected point clouds
% %selectedIndices = [3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 19, 22, 23, 24, 26, 27, 28, 29, 30, 79, 80, 81, 82, 88, 89, 90, 93, 104, 106, 107, 108, 127, 131, 134, 135, 136, 137, 138, 140, 150, 152, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 190, 191];
% 
% %selectedIndices = [1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,19,22,23,24,26,27,28,29,30,79,80,81,82,84,85,86,88,89,90,93,95,104,105,106,108,108,127,131,134,135,136,137,138,140,149,150,152,153,157,158,159,160,161,162,163,164,165,166,167,168,169,170,171,172,174,175,176,177,178,179,180,181,182,183,184,185,186,187,188,190,191,196,201];
% 
% %points 196 and 201 show promise and have sides of the book. will need to
% %remove the interference 
% 
% %selectedIndices = 1:213;
% % Initialize an empty cell array to store the selected point clouds
% selectedPointClouds = cell(1, numel(selectedIndices));
% 
% 
% %% Read TSV file 
% t = readtable("TestRandT.txt", "FileType","text",'Delimiter', '\t');
% %find xyz points 
% 
% % tData = zeros(70,3);
% % tData(:,1) = t(:,10);
% 
% tData = zeros(80,3);
% tData(:,1) = t.Rx;
% tData(:,3) = t.Ry;
% tData(:,5) = t.Rz;
% 
% % for i = 1:70
% %     figure;
% %     plot3(tData(i,1),tData(i,2),tData(i,3), 'ro'); % 'ro' represents red circles
% %     hold on;
% % end
% axis equal;
% 
% %% Define the camera intrinsics for all depth images (outside the loop)
% intrinsicsList = cell(1, depthTopicMessageNum);
% for k = 1:depthTopicMessageNum
%     imageSize = size(depthImagesOut{k}, [1, 2]);
%     intrinsicsList{k} = cameraIntrinsics(focalLength, principalPoint, imageSize);
% end
% 
% %% Iterate through the selected point clouds and store them in the cell array
% for i = 1:numel(selectedIndices)
%     index = selectedIndices(i);
%     depthImage = readImage(depthImagesOut{index});
% 
%     % Calculate intrinsics for the current depth image using the correct index
%     imageSize = size(depthImage, [1, 2]);
%     intrinsics = intrinsicsList{index};
% 
%     % Convert the depth image to a 3D point cloud using the correct intrinsics
%     pointCloud = createPointCloud(depthImage, intrinsics, depthScaleFactor);
% 
%     %% Point Cloud Rotation and Translation
%     % %this is for translating and rotating point clouds needs to be done for
%     % %each
%     % rotationAngles = [tData(i,1),tData(i,2),tData(i,3)]; %this is wrong just random rotation of tsv we need to calibrate
%     % translation = [0 0 0];
%     % tform = rigidtform3d(rotationAngles,translation);
%     % 
%     % %Transform each point cloud.
%     % ptCloudOut = pctransform(pointCloud,tform);
% 
%     % Store the point cloud in the selectedPointClouds array
%     selectedPointClouds{i} = pointCloud;
% 
% end
% 
% %% Initialize the master point cloud with the first selected point cloud
% 
% fixedSample = pcdownsample(selectedPointClouds{1},'random',0.1);
% 
% 
% for i = 2:length(selectedPointClouds)
% 
%     movingSample = pcdownsample(selectedPointClouds{i},'random',0.1);
% 
%     [~,movingReg] = pcregistericp(movingSample,fixedSample,Metric="PlaneToPlane");
% 
%     fixedSample = movingReg;
% 
% end
% 
% pcshow(fixedSample);
% 

%figure;
%pcshowpair(movingReg,selectedPointClouds{1},'VerticalAxis','Y','VerticalAxisDir','Down');

% %% Merge the remaining selected point clouds one by one
% for i = 2:numel(selectedIndices)
%     masterPointCloud = pcmerge(masterPointCloud, selectedPointClouds{i}, 0.001); % You can adjust the mergeSize if needed this will change point density 
% end
% 
% %% Visualize the master point cloud
% figure;
% pcshow(masterPointCloud, 'VerticalAxis', 'Y', 'VerticalAxisDir', 'Up', 'ViewPlane', 'YX');
% title('Selected Merged Point Cloud');
% xlabel('X (m)');
% ylabel('Y (m)');
% zlabel('Z (m)');
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

% Visualize all point clouds (optional)
% figure;
% pcshow(pointCloudFunc);
% title('3D Point Cloud');
% xlabel('X (m)');
% ylabel('Y (m)');
% zlabel('Z (m)');
end



%% Region of Interest (ROI) - WIP
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