clc;
clear;
close all;

tran = [-0.3; -0.3; 0];
h = 2;
feature_i = [0 0 0; 10 0 0; 10 3 0; 3 3 0; 3 6 0;...
    0 0 h; 10 0 h; 10 3 h; 3 3 h; 3 6 h] + tran';
feature_ = [3 6 0; 10 6 0; 10 10 0; 0 10 0;0 0 0; ...
    3 6 h; 10 6 h; 10 10 h; 0 10 h;0 0 h; ] + tran';

tri1 = [1 2 6; 2 6 7; 2 3 7; 3 7 8; 3 4 8; 4 8 9; 4 5 9; 5 9 10];
tri_ = [1 2 6; 2 6 7; 2 3 7; 3 7 8; 3 4 8; 4 8 9; 4 5 9; 5 9 10];

plot3(feature_i(:, 1), feature_i(:, 2), feature_i(:, 3), 'b.');
hold on;
for i = 1:length(feature_i)
    s = num2str(i);
    text(feature_i(i, 1)+0.1, feature_i(i, 2)+0.1, feature_i(i, 3)+0.1, s, 'color', 'r');
end
plot3(feature_(:, 1), feature_(:, 2), feature_(:, 3), 'b.');
hold on;
for i = 1:length(feature_)
    s = num2str(i);
    text(feature_(i, 1)-0.1, feature_(i, 2)-0.1, feature_(i, 3)-0.1, s, 'color', 'r');
end
% figure(2)
trisurf(tri1, feature_i(:, 1), feature_i(:, 2), feature_i(:, 3));
trisurf(tri_, feature_(:, 1), feature_(:, 2), feature_(:, 3));


trans = [-0.3; -0.3; 0];
h2 = 2;
feature = [0 0 0; 10 0 0; 10 3 0; 3 3 0; 3 6 0; 10 6 0; 10 10 0; 0 10 0;...
    0 0 h; 10 0 h; 10 3 h; 3 3 h; 3 6 h; 10 6 h; 10 10 h; 0 10 h;] + trans';
feature = feature';

M = length(feature(1, :));
id1 = 1:length(feature_i(:, 1));
vertex = feature_i';
% feature_ = [10 40 6; 15 30 6; 0 40 6; 0 30 6]';

goalx = -0.5:0.5:10;
goaly = -0.5:0.5:10;
goalz = [0.5,2];
% goalx = -1:3:26;
% goaly = -1:3:71;
% goalz = 2:2:6;
unknown = [];
for x = 1:length(goalx)
    for y = 1:length(goaly)
        for z = 1:length(goalz)
            unknown = [unknown, [x; y; z]];
        end
    end
end
save dataFeature2 M feature id1 tri1 goalx goaly goalz unknown feature_ tri_ vertex
% 
% axis([-4 4 -4 4 -4 4]);