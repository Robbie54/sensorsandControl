%read tsv file

t = readtable("forCali_GroundTruth_new360.tsv", "FileType","text",'Delimiter', '\t');

%find xyz points 

% tData = zeros(70,3);
% tData(:,1) = t(:,10);


tData = zeros(70,3);
tData(:,1) = t.Tx;
tData(:,2) = t.Ty;
tData(:,3) = t.Tz;

figure;
hold on;

for i = 1:70
    plot3(tData(i,1),tData(i,2),tData(i,3), 'ro'); % 'ro' represents red circles
end
axis equal;

%%
%read calibration rosbag
bag = rosbag('CalibNew_360.bag');

%information extracted from ndi_aurora topic

%this could possibly be the end effector location
quaternion = [0.6004000306129456,-0.48580002784729,-0.5668999552726746,0.2864000201225281];
translation = [-12.0600004196167,31.46999931335449,-316.0299987792969];
