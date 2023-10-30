%% File Setup
clear all
clf
close all
clc

%% Rosbag Depth Reading
bag = rosbag('Modelnew2_360.bag');
depthTopic = select(bag, 'Topic', '/camera/depth/image_rect_raw');
allDepthImages = readMessages(depthTopic);
depthTopicMessageNum = depthTopic.NumMessages;
depthImagesOut = readMessages(depthTopic);

%% Depth camera intrinsics extracted from ModelNewBag topic /camera/depth/camera_info
K = [421.7674560546875, 0, 423.2069396972656, 0, 421.7674560546875, 239.1299591064453, 0, 0, 1];
focalLength = K(1);
principalPoint = [K(3), K(6)]; %x and y respectively
depthScaleFactor = 5e3;

%% Define the indices of the selected point clouds
% All available point clouds once interference has been filtered out.
selectedIndices = [1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,60,61,62,65,66,68,69,73,74,75,78,79,80,81,82,83,84,85,86,87,88,89,90,93,94,95,96,97,98,99,100,101,102,103,104,105,106,107,108,109,110,111,112,113,114,115,116,117,118,119,120,121,122,123,124,127,131,134,135,136,137,138,139,140,141,142,143,144,145,146,147,148,149,150,151,152,153,154,155,156,157,158,159,160,161,162,163,164,165,166,167,168,169,170,171,172,173,174,175,176,177,178,179,180,181,182,183,184,185,186,187,188,189,190,191,192,193,194,195,196,197,200,201,204,206,207,208,209,210,211,212];

% Selects Only a Limited Number of Available Point Clouds to Improve Performance.
everyxNumber = selectedIndices(1:6:end);

% Initialize an empty cell array to store the selected point clouds.
selectedPointClouds = cell(1, numel(everyxNumber));

%% Iterate through depth images 51-100 and create a point cloud
pointClouds = cell(1, length(everyxNumber));
Cloud = cell(1, length(everyxNumber));
roi = [-0.04 0.035 -0.02 0.1 0 0.08];
for k = 1:length(everyxNumber)
    depthImage = readImage(allDepthImages{k});

    % Calculate intrinsics for the current depth image.
    imageSize = size(depthImage, [1, 2]);
    intrinsics = cameraIntrinsics(focalLength, principalPoint, imageSize);

    % Convert the depth image to a 3D point cloud.
    pointCloud = createPointCloud(depthImage, intrinsics, depthScaleFactor);

    %Apply ROI filter to the current point cloud.
    indices = findPointsInROI(pointCloud, roi);
    roiPointCloud = select(pointCloud,indices);

    % Store the point cloud in the cell array.
    pointClouds{k} = roiPointCloud;
    Cloud{k} = roiPointCloud;
end

%% Generates the Final Point Cloud
% Initialises Cell Array Variables for Storage.
t = cell(1,length(everyxNumber));
Apc = cell(1,length(everyxNumber));
t{1} = pcregistericp(pointClouds{2},pointClouds{1});
Apc{1} = pctransform(pointClouds{2},t{1});

% Transforms all Point Clouds to Form a Final Point Cloud.
for i = 2:length(selectedPointClouds)
    Cloud{i} = pcdownsample(pointClouds{i},'gridAverage',0.0001);
    t{i} = pcregistericp(Cloud{i},Cloud{1});
    endCloud{i} = pcdownsample(pointClouds{i}, 'random', 0.2);
    Apc{i} = pctransform(endCloud{i},t{i});
    hold on;
    pcshow(Apc{i});
end

%% Create Point Cloud Function
function pointCloudFunc = createPointCloud(depthImage, intrinsics, depthScaleFactor)
[rows, cols] = size(depthImage);

% Initialize arrays to store X, Y, and Z coordinates.
X = zeros(rows, cols);
Y = zeros(rows, cols);
Z = zeros(rows, cols);

% Apply intrinsics and depth scale factor to convert to 3D coordinates.
for r = 1:rows
    for c = 1:cols
        Z(r, c) = double(depthImage(r, c)) / depthScaleFactor;
        X(r, c) = (c - intrinsics.PrincipalPoint(1)) * Z(r, c) / intrinsics.FocalLength(1);
        Y(r, c) = (r - intrinsics.PrincipalPoint(2)) * Z(r, c) / intrinsics.FocalLength(2);
    end
end

% Create a point cloud using the pointCloud constructor without colour.
XYZ = cat(3, X, Y, Z);
pointCloudFunc = pointCloud(XYZ);
end