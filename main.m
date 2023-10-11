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
    
%% Depth show
     
    bag = rosbag('Modelnew2_360.bag');
    depthTopic = select(bag, 'Topic', '/camera/depth/image_rect_raw');  
    firstDepthImage = readMessages(depthTopic,1); %can get all 
    secondDepthImage = readMessages(depthTopic,2); %can get all 

    formatedDepthImage = readImage(firstDepthImage{1}); %some how readImage works was meant to be discontinued and now rosReadImage
    formatedDepthImage2 = readImage(secondDepthImage{1});

    imshow(uint8((double(formatedDepthImage)/350)*255)) %since this is a depth image we need to scale to be within uint8 range to display colour 350 was chosen based on largest number in data set
   
    
    %these are now correct unless they change between messages 
    %http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/CameraInfo.html
        focalLength      = [610.339, 609.110]; %taken from topic /camera/color/camera_info message 1 
        principalPoint   = [317.109, 228.684];

        %NOT CORRECT intrinsics taken from help page need to find camera info 
        imageSize        = size(formatedDepthImage,[1,2]);
        intrinsics       = cameraIntrinsics(focalLength,principalPoint,imageSize);
        depthScaleFactor = 5e3;
        maxCameraDepth   = 5;

    %%Point cloud 
    ptCloud = pcfromdepth(formatedDepthImage, depthScaleFactor, intrinsics); 
    ptCloud2 = pcfromdepth(formatedDepthImage2, depthScaleFactor, intrinsics); 

    pcshow(ptCloud, VerticalAxis="Y", VerticalAxisDir="Up", ViewPlane="YX");

    ptCloudRef = ptCloud;
    ptCloudCurrent = ptCloud2;
    gridSize = 0.1;
    
    %these might be wehre we put the em sensor trans and rot 
    fixed = pcdownsample(ptCloudRef,gridAverage=gridSize);
    moving = pcdownsample(ptCloudCurrent,gridAverage=gridSize);

    tform = pcregistericp(moving,fixed,Metric="pointToPlane");
    ptCloudAligned = pctransform(ptCloudCurrent,tform);

    mergeSize = 0.015;
    ptCloudScene1 = pcmerge(ptCloudRef,ptCloudAligned,mergeSize);

    % Visualize the input images.
    figure
    subplot(2,2,1)
    imshow(ptCloudRef.Color)
    title("First input image",Color="w")

    subplot(2,2,3)
    imshow(ptCloudCurrent.Color)
    title("Second input image",Color="w")

    % Visualize the world scene.
    subplot(2,2,[2,4])
    pcshow(ptCloudScene1,VerticalAxis="Y",VerticalAxisDir="Down")
    title("Initial world scene")
    xlabel("X (m)")
    ylabel("Y (m)")
    zlabel("Z (m)")





%% The below section is from the link on how to open a point cloud use for reference but not useful for bags 
    %%From 
    %%https://au.mathworks.com/help/vision/ug/3-d-point-cloud-registration-and-stitching.html?fbclid=IwAR3Amoezr36uL9f3yXKkBhPA5fCq7WxHiYWApa7ps96OtMsiiAO91k1QZe4
    fullfile(toolboxdir("vision"))
    dataFile = fullfile(toolboxdir("vision"),"visiondata","livingRoom.mat"); %this data is in a very different format to any one topic,
                                                                               %we need to work out what data from which topics we need
                                                                              

    load(dataFile);

    % Extract two consecutive point clouds and use the first point cloud as
    % reference.
    ptCloudRef = livingRoomData{1};
    ptCloudCurrent = livingRoomData{2};
    gridSize = 0.1;
    fixed = pcdownsample(ptCloudRef,gridAverage=gridSize);
    moving = pcdownsample(ptCloudCurrent,gridAverage=gridSize);

    tform = pcregistericp(moving,fixed,Metric="pointToPlane");
    ptCloudAligned = pctransform(ptCloudCurrent,tform);

    mergeSize = 0.015;
    ptCloudScene1 = pcmerge(ptCloudRef,ptCloudAligned,mergeSize);

    % Visualize the input images.
    figure
    subplot(2,2,1)
    imshow(ptCloudRef.Color)
    title("First input image",Color="w")

    subplot(2,2,3)
    imshow(ptCloudCurrent.Color)
    title("Second input image",Color="w")

    % Visualize the world scene.
    subplot(2,2,[2,4])
    pcshow(ptCloudScene1,VerticalAxis="Y",VerticalAxisDir="Down")
    title("Initial world scene")
    xlabel("X (m)")
    ylabel("Y (m)")
    zlabel("Z (m)")
    








