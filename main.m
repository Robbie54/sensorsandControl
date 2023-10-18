%%Assignment 1

%removing functions as they shouldn't be in main messes the workspace up
%need to make a bag class instead
clear
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

%% Depth show with loops
bag = rosbag('Modelnew2_360.bag');
depthTopic = select(bag, 'Topic', '/camera/depth/image_rect_raw');
depthTopicMessageNum = depthTopic.NumMessages;

depthImagesOut = readMessages(depthTopic);

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

% Initialize an empty cell array to store point clouds
formatedDepthImages = cell(1, depthTopicMessageNum);
pointClouds = cell(1, depthTopicMessageNum);

% Initialize an empty point cloud to accumulate all point clouds
masterPointCloud = pointCloud(zeros(0, 3));

for i = 1:depthTopicMessageNum
    formatedDepthImage = readImage(depthImagesOut{i});
    
    % Calculate intrinsics for each depth image
    imageSize = size(formatedDepthImage, [1, 2]);
    focalLength = [610.339, 609.110]; % Adjust these values as needed
    principalPoint = [317.109, 228.684]; % Adjust these values as needed
    intrinsics = cameraIntrinsics(focalLength, principalPoint, imageSize);
    depthScaleFactor = 5e3;
    
    % Convert the depth image to a point cloud
    pointCloud = pcfromdepth(formatedDepthImage, depthScaleFactor, intrinsics);
    
    % Accumulate the current point cloud into the master point cloud
    masterPointCloud = pcmerge(masterPointCloud, pointCloud, 0.01); % Adjust the threshold as needed
end

% Visualize the accumulated master point cloud
figure;
pcshow(masterPointCloud, 'VerticalAxis', 'Y', 'VerticalAxisDir', 'Up', 'ViewPlane', 'YX');
title('Master Point Cloud');
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');


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
% 
% 
