function pose = getPose(posedata)
a = posedata.Pose(end);
rx = a.Position.X;
ry = a.Position.Y;
rz = a.Position.Z;

ox = a.Orientation.X;
oy = a.Orientation.Y;
oz = a.Orientation.Z;
ow = a.Orientation.W;

roll = atan2(2*(ow*ox + oy*oz), 1-2*(ox*ox + oy*oy));
pitch = asin(2*(ow*oy - oz*ox));
yell = atan2(2*(ow*oz + ox*oy), 1-2*(oz*oz + oy*oy));
yell = wrapToPi(yell);
pose = [0; 0; yell; rx; ry; rz];
end

