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
selectedIndices = [3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 19, 22, 23, 24, 26, 27, 28, 29, 30, 79, 80, 81, 82, 88, 89, 90, 93, 104, 106, 107, 108, 127, 131, 134, 135, 136, 137, 138, 140, 150, 152, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 190, 191];

% Initialize an empty cell array to store the selected point clouds
selectedPointClouds = cell(1, numel(selectedIndices));

%% Iterate through depth images 51-100 and create a point cloud
pointClouds = cell(1, depthTopicMessageNum);
roi = [-0.04 0.035 -0.02 0.1 0 0.08]; %roi = [-0.1 0.1 -0.05 1 0 0.2];
for k = 1:depthTopicMessageNum
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
    pointClouds{k} = roiPointCloud;
end



% % Visualize specific point clouds
% for k = [1, 201]
%     figure;
%     pcshow(pointClouds{k}, 'VerticalAxis', 'Y', 'VerticalAxisDir', 'Down');
%     title(['Point Cloud ', num2str(k)]);
%     xlabel('X (m)');
%     ylabel('Y (m)');
%     zlabel('Z (m)');
% end

%% Initialize the master point cloud with the first selected point cloud

fixedSample = pcdownsample(pointClouds{1},'random',0.1);

figure;
pcshow(fixedSample);
title('down sampled point cloud 1');

%for i = 2:length(selectedPointClouds)



    movingSample = pcdownsample(pointClouds{201},'random',0.1);


    figure;
    variable = pointClouds{201};
    pcshow(variable);

    figure;
    pcshow(movingSample);
    title('down sampled point cloud 201')
    
    t = pcregistericp(fixedSample,movingSample);

    [t,movingReg] = pcregistericp(fixedSample,movingSample);
    figure;
    pcshow(movingReg);
%%
    % figure;
    % pcshow(movingReg);
    % title('merged point cloud');

%end




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
% %% Depth camera intrinsics extracted from ModelNewBag topic /camera/depth/camera_info
% K = [421.7674560546875, 0, 423.2069396972656, 0, 421.7674560546875, 239.1299591064453, 0, 0, 1];
% roi = [-0.04 0.035 -0.02 0.1 0 0.08]; %roi = [-0.1 0.1 -0.05 1 0 0.2];
% focalLength = K(1);
% principalPoint = [K(3),K(6)]; %x and y respectively 
% depthScaleFactor = 5e3; %experimental depth svale factor value set 
% 
% 
% %% Define the indices of the selected point clouds
% 
% %not all depth valuses provide useful information. Lots of interference and
% %noise can be found in these images, as well as outlying artefacts. these
% %must be filtered out manually. for the outying artefacts, they can be
% %cropped out to include the important data.
% 
% 
% %selectedIndices = [3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 19, 22, 23, 24, 26, 27, 28, 29, 30, 79, 80, 81, 82, 88, 89, 90, 93, 104, 106, 107, 108, 127, 131, 134, 135, 136, 137, 138, 140, 150, 152, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 190, 191];
% %selectedIndices = [1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,60,61,62,65,66,68,69,73,74,75,78,79,80,81,82,83,84,85,86,87,88,89,90,93,94,95,96,97,98,99,100,101,102,103,104,105,106,107,108,109,110,111,112,113,114,115,116,117,118,119,120,121,122,123,124,127,131,134,135,136,137,138,139,140,141,142,143,144,145,146,147,148,149,150,151,152,153,154,155,156,157,158,159,160,161,162,163,164,165,166,167,168,169,170,171,172,173,174,175,176,177,178,179,180,181,182,183,184,185,186,187,188,189,190,191,192,193,194,195,196,197,200,201,204,206,207,208,209,210,211,212,213];
% 
% 
% %not all depth images can be selected to allign and merge into the master
% %point cloud. Because we are down sampling, certain data sets that provide
% %unique information from the other frames, will slowly be filterd out when
% %downsampling all frames to keep the point cloud count down.
% selectedIndices = [1,50,117,118,196,201]; % best values 
% 
% % Initialize an empty cell array to store the selected point clouds
% selectedPointClouds = cell(1, numel(selectedIndices));
% 
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
%     %Apply ROI filter to the current point cloud
%     indices = findPointsInROI(pointCloud, roi);
%     roiPointCloud = select(pointCloud,indices);
% 
% 
%     % Store the point cloud in the selectedPointClouds array
%     selectedPointClouds{i} = pointCloud;
% 
% end
% 
% %% Initialize the master point cloud with the first selected point cloud
% 
% fixedSample = pcdownsample(selectedPointClouds{1},'random',0.5);
% 
% 
% for i = 2:length(selectedPointClouds)
% 
%     movingSample = pcdownsample(selectedPointClouds{i},'random',0.5);
% 
%     [~,movingReg] = pcregistericp(movingSample,fixedSample,Metric="PlaneToPlane");
% 
%     fixedSample = movingReg;
% 
% end
% 
% pcshow(fixedSample);


% Create Point Cloud Function
    function pointCloudFunc = createPointCloud(depthImage, intrinsics, depthScaleFactor)
    [rows, cols] = size(depthImage);

    %Initialize arrays to store X, Y, and Z coordinates
    X = zeros(rows, cols);
    Y = zeros(rows, cols);
    Z = zeros(rows, cols);

    %Apply intrinsics and depth scale factor to convert to 3D coordinates
    for r = 1:rows
        for c = 1:cols
            Z(r, c) = double(depthImage(r, c)) / depthScaleFactor;
            X(r, c) = (c - intrinsics.PrincipalPoint(1)) * Z(r, c) / intrinsics.FocalLength(1);
            Y(r, c) = (r - intrinsics.PrincipalPoint(2)) * Z(r, c) / intrinsics.FocalLength(2);
        end
    end

    %Create a point cloud using the pointCloud constructor without color
    XYZ = cat(3, X, Y, Z);
    pointCloudFunc = pointCloud(XYZ);

    % %Visualize all point clouds (optional)
    % figure;
    % pcshow(pointCloudFunc);
    % title('3D Point Cloud');
    % xlabel('X (m)');
    % ylabel('Y (m)');
    % zlabel('Z (m)');
    end



% 
% 
% %% BElow is commit for the rot and transl does not use above code which is duplicate
%         % calibBag = rosbag('CalibNew_360.bag');
%         % calibTopic = select(calibBag, 'Topic', '/ndi_aurora');
%         % calibTopicMessageNum = calibTopic.NumMessages;
%         % calibMessages = readMessages(calibTopic, 'DataFormat', 'struct');
% %% Define the camera intrinsics for all depth images (outside the loop)
%         % intrinsicsList = cell(1, depthTopicMessageNum);
%         % for k = 1:depthTopicMessageNum
%         %     imageSize = size(depthImagesOut{k}, [1, 2]);
%         %     intrinsicsList{k} = cameraIntrinsics(focalLength, principalPoint, imageSize);
%         % end
%         % 
%         % %% Iterate through the selected point clouds and store them in the cell array
%         % for i = 1:numel(selectedIndices)
%         %     index = selectedIndices(i);
%         %     depthImage = readImage(depthImagesOut{index});
%         % 
%         %     % Calculate intrinsics for the current depth image using the correct index
%         %     imageSize = size(depthImage, [1, 2]);
%         %     intrinsics = intrinsicsList{index};
%         % 
%         %     % Convert the depth image to a 3D point cloud using the correct intrinsics
%         %     pointCloud = createPointCloud(depthImage, intrinsics, depthScaleFactor);
%         % end
%         % 
%         %     %% Point Cloud Rotation and Translation
%         %     %this is for translating and rotating point clouds 
%         %     % it works quite well can chose to eith just do maxs selected clouds or all of them 
%         %     %thing is all of them the rotation is slightly off since there
%         %     %is 546 calib messages and only 213 depth not sure
%         % 
%         %     hold on;
%         % for i = 1:50
%         % 
%         %     rotation = calibMessages{i}.PortHandlePoses.Rotation;
%         %     translation = calibMessages{i}.PortHandlePoses.Translation;
%         % 
%         % 
%         %     % Extract the quaternion components
%         %     rotationX = rotation.X;
%         %     rotationY = rotation.Y;
%         %     rotationZ = rotation.Z;
%         %     rotationW = rotation.W;
%         % 
%         %     rot = [rotationX rotationY rotationZ rotationW];
%         % 
%         %     translationX = translation.X;
%         %     translationY = translation.Y;
%         %     translationZ = translation.Z;
%         % 
%         %     rotRot = quat2rotm(rot);
%         % 
%         % 
%         %     % rotationAngles = [rotRot.X,rotRot.Y,rotRot.Z]; 
%         %     translation = [0,0,0];
%         %     tform = rigidtform3d(rotRot,translation);
%         % 
%         %     %Transform each point cloud.
%         %     ptCloudOut = pctransform(pointCloud,tform);
%         % 
%         %     % Store the point cloud in the selectedPointClouds array
%         %     selectedPointClouds{i} = ptCloudOut;
%         % 
%         %      lowResPointCloud = pcdownsample(selectedPointClouds{i}, 'nonuniformGridSample', 99);
%         %     %   figure;
%         %     % pcshow(lowResPointCloud);
%         %     % title('3D Point Cloud');
%         %     % xlabel('X (m)');
%         %     % ylabel('Y (m)');
%         %     % zlabel('Z (m)');
%         % 
%         %      pcshow(lowResPointCloud,'VerticalAxis','Y','VerticalAxisDir','Down');
%         %     hold on;
%         % end
% 
% %% END my commit 