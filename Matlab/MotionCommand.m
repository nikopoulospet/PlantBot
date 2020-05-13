function [omega,crab_angle] = MotionCommand()
%body_v = 0.04; %m/s center of gravity
theta = pi/6; % constamt angle. 30 deg. even between leg 1 and 2
omega = 0.3;%no rotational velocity atm.
heading = pi/3; %inital pose
crab_angle = heading - theta;

end

