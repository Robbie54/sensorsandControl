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