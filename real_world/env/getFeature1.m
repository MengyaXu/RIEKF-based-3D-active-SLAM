clc;
clear;
close all;
M = 20;
feature = zeros(3,M);
feature(:,3) = [3; 0.561; 0.16];
feature(:,5) = [3; 1.357; 0.162];
feature(:,6) = [3; 2.845; 0.165];
feature(:,7) = [0; 0.21; 0.165];
feature(:,8) = [0.214; 3; 0.166];
feature(:,17) = [1.379; 0; 0.154];
feature(:,18) = [1.422; 3; 0.154];

ide = [3 5 6 7 8 17 18];

%%
goalx = -0.15:0.3:3.2;
goaly = -0.15:0.3:3.2;
goalz = 0;
% goalx = -1:3:26;
% goaly = -1:3:71;
% goalz = 2:2:6;
unknown = [];
for x = 1:length(goalx)
    for y = 1:length(goaly)
        for z = 1:length(goalz)
            unknown = [unknown, [x; y]];
        end
    end
end
save dataFeature1 M feature ide goalx goaly goalz unknown
% 
% axis([-4 4 -4 4 -4 4]);