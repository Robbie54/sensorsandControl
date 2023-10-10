%%Assignment 1 
classdef main
    methods
    
    %%functions control 
    function self = main()
        
        close all
        clc

        %toggle what you want by toggeling 
        % self.dobot()
         %self.bagTest()
        self.depthBag()
    end
    
    
    % set(0,'DefaultFigureWindowStyle','docked')
    
    %% Create Dobot
    
    % Displays the status
    function dobot(self)
        disp('Please wait creating dobot... (milo still throwing)');
        
        % Creates the dobot
        createDobot = Dobot();
        
        hold on;
        camlight;
        
        % Displays the status
        disp('Dobot Created yay, func end');
        pause
    end
    
    %% Bag
    %currently code just selects single message from camera topic and displays
    %image 
    %there must be a better way 
    
    %I am unsure how to use the depth data for anything useful 
    function bagTest(self)
        bag = rosbag('Modelnew2_360.bag');
        
        rosbag info 'Modelnew2_360.bag';
        % rosbag info 'CalibNew_360.bag';
        
        imageRaw = select(bag, 'Topic', '/camera/color/image_raw');
        
        
        message = readMessages(imageRaw,1); %read topic (message 1)

        %we need a better way then this to access the data and use it 

        % imageData = message.Data;
        width = message{1}.Height; %get specific data from 1st message in topic 
        height = message{1}.Width;
        
        imageData = message{1}.Data;
        
        R = imageData(1:3:end,:); %take one number every three from the data to get r data 
        G = imageData(2:3:end,:);
        B = imageData(3:3:end,:);
        
        R = reshape(R, height,width); %reshape for imsho 
        G = reshape(G, height,width);
        B = reshape(B, height,width);
        
        
        RGB = cat(3, R, G, B); %concatonate 
        disp(' pause continuing will close dobot and open image figure ');
        
        pause;
        close all
        hold on;
        imshow(RGB);
    end

%%Depth 
%Next function i want to show the depth topic in matlab then see if i can
%overlay it in the dobot plot 
%weirdly the depth data is same size as rgb so seems to be rgb? idk 
function depthBag(self)
    bag = rosbag('Modelnew2_360.bag');
    depthTopic = select(bag, 'Topic', '/camera/depth/image_rect_raw');  
    message = readMessages(depthTopic,1); % this is wron
    disp('egg')

       
    
end

    end
end






