function [xOff, yOff] = firstFrameOffset

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis



%% first frame transformation
% trans = [967.74532728940051 -125.03844593074048 -825.44541012231355];  % mm
% ang = 0.31012228973834965;  % rad
% ax = [1.7218947361010425e-15 1 -1.1267623945645298e-14];

trans = [-2.8761710264867659 4.7407179135113173 -358.27937645160171];  % mm
ang = 0.1044100976967795;  % rad
ax = [-3.7005803898906935e-14 -1 -8.2222525660183538e-14];

%convert from any angle rotation to a rotation matrix
axang = [ax, ang];
rotm = axang2rotm(axang);
%from rotation matrix to homogenous transformation matrix
homoT = rotm2tform(rotm);
homoT(1:3,4) = trans'; 
u = [1 1 1 1]';


totalOffset = homoT * u;

xOff = totalOffset (1)/1000; %from mm to m
yOff = totalOffset (2)/1000;
%%
end