%% File Setup
clf
close all
clear all
clc

%% Rosbag Depth Reading
bag = rosbag('Modelnew2_360.bag');
depthTopic = select(bag, 'Topic', '/camera/depth/image_rect_raw');
depthTopicMessageNum = depthTopic.NumMessages;
depthImagesOut = readMessages(depthTopic);

%% Define camera parameters
focalLength = [610.339, 609.110]; % Adjust these values as needed
principalPoint = [317.109, 228.684]; % Adjust these values as needed
depthScaleFactor = 5e3;

%% Define the indices of the selected point clouds
selectedIndices = [3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 19, 22, 23, 24, 26, 27, 28, 29, 30, 79, 80, 81, 82, 88, 89, 90, 93, 104, 106, 107, 108, 127, 131, 134, 135, 136, 137, 138, 140, 150, 152, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 190, 191];

% Initialize an empty cell array to store the selected point clouds
selectedPointClouds = cell(1, numel(selectedIndices));

%% Define the camera intrinsics for all depth images (outside the loop)
intrinsicsList = cell(1, depthTopicMessageNum);
for k = 1:depthTopicMessageNum
    imageSize = size(depthImagesOut{k}, [1, 2]);
    intrinsicsList{k} = cameraIntrinsics(focalLength, principalPoint, imageSize);
end

%% Iterate through the selected point clouds and store them in the cell array
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

%% Initialize the master point cloud with the first selected point cloud
masterPointCloud = selectedPointClouds{1};

%% Merge the remaining selected point clouds one by one
for i = 2:numel(selectedIndices)
    masterPointCloud = pcmerge(masterPointCloud, selectedPointClouds{i}, 0.001); % You can adjust the mergeSize if needed
end

%% Visualize the master point cloud
figure;
pcshow(masterPointCloud, 'VerticalAxis', 'Y', 'VerticalAxisDir', 'Up', 'ViewPlane', 'YX');
title('Selected Merged Point Cloud');
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');

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

%% Set axes limits
% xlim([-1, 1]); % Set the limits for the X-axis
% ylim([-1, 1]); % Set the limits for the Y-axis
% zlim([0, 2]); % Set the limits for the Z-axis
