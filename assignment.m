%%Assignment 1 

clear all 
close all 

bag = rosbag('Modelnew2_360.bag');

% rosbag info 'Modelnew2_360.bag';
% rosbag info 'CalibNew_360.bag';

imageRaw = select(bag, 'Topic', '/camera/color/image_raw');


message = readMessages(imageRaw,1); %read topic (message 1)
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
pause;
hold on;
imshow(RGB);






