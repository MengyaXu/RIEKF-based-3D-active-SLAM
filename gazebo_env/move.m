function [u, ur] = move(u, ur,robot, velmsg, scScrib)
% odomdata0 = receive(odomScrib,10);
% odom0 = getPose(odomdata0.Pose);

dw = pi/12;
dv = 0.2;

w = ur(3);
v = ur(4);
tw1 = w/dw;
tw = abs(w/dw);
tic
while toc <= tw
    velmsg.Angular.Z = (tw1/tw) * dw;
    velmsg.Linear.X = 0;
    send(robot, velmsg);
end

if v > 0
    scdata = receive(scScrib,10);
    scan = lidarScan(scdata);
    ranges = scan.Ranges;
    ranges(isnan(ranges)) = scdata.RangeMax;
    a = find(ranges <= 0.01);
    ranges(a) = [];
    angles = scan.Angles;
    angles(a) = [];
    
    [angle_min, angle_id] = min(abs(angles));
    id1 = max(1, angle_id - 60);
    id2 = min(length(angles), angle_id+60);
    rg = ranges(id1:id2);
    dmin = min(rg - v);
    if dmin > 0.1 && min(ranges) > 0.1
        tv1 = v/dv;
        tv = abs(v/dv);
        tic
        while toc <= tv
            velmsg.Angular.Z = 0;
            velmsg.Linear.X = (tv1/tv) * dv;
            send(robot, velmsg);
        end
    else
        ur(4) = 0;
        u(4:6) = [0;0;0];
    end
end
end

