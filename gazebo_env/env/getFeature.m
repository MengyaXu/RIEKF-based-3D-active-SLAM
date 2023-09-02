function feature = getFeature(posedata, id_f, pd)
feature = [];
b = posedata.Pose;
for i = 3:length(b)-1
    a = b(i);
    rx = a.Position.X;
    ry = a.Position.Y;
    rz = a.Position.Z;
    feature = [feature, [rx; ry; rz]];
end

x_min = min(feature(1,:));
x_max = max(feature(1,:));
y_min = min(feature(2,:));
y_max = max(feature(2,:));
z_min = min(feature(3,:));
z_max = max(feature(3,:));

goalx = x_min-0.5:pd:x_max+0.5;
goaly = y_min-0.5:pd:y_max+0.5;
goalz = [z_min,z_max];
unknown = [];
for x = 1:length(goalx)
    for y = 1:length(goaly)
        unknown = [unknown, [x; y]];
    end
end
save(['dataFeature' num2str(id_f)], 'goalx','goaly','goalz', 'unknown');