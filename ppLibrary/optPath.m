function [TrajSol] = optPath(waypoints, distance, map)
v = 10; 
%This function smoothes the reference path given by waypoints 

NumSamples = round((distance / (v/3.6)) * 10);

%% get the non optimise path
x = waypoints(:,1);
y = waypoints(:,2);

xway = x;
yway = y;

for i = 1:length(x)-1
    heading (i,1) = atan2( (y(i+1)-y(i)), (x(i+1)-x(i)) ); %theta
end

heading= [heading; heading(end)];

nonOptPath = [x, y, heading];

%% set options
options = optimizePathOptions;
options.MinTurningRadius = 3.5; % meters
options.MaxVelocity = 6; % m/s
options.MaxAcceleration = 2; % m/s/s
options.ReferenceDeltaTime = 0.15; % second
options.MaxPathStates = NumSamples; 
options.ObstacleSafetyMargin = 1;
options.WeightSmoothness = 1000;
options.WeightVelocity = 1;
options.WeightTime = 20;
options.WeightMinTurningRadius = 10;
options.WeightObstacles = 20;
options.WeightAcceleration = 1;
options.WeightAngularVelocity = 1;
options.WeightAngularAcceleration = 1;

%% optimize the trajectory
[optpath, kinInfo, solinfo] = optimizePath(nonOptPath ,map, options);


q=[];
q(:,1) = optpath(:,1)';
q(:,2) = optpath(:,2)';
refVel = [kinInfo.Velocity;0]'; %, kinInfo

timeStamp = kinInfo.TimeStamps';


%% Resampling
Ts = 0.1; %10Hz

addToEnd = 20;

%Remove first element of the signals from the signal, so the signals starts
%from 0, this is compensated for after resampling.

x0 = q(:,1) - q(1,1); 
y0 = q(:,2) - q(1,2);

% we add some dummy values at the end of the signal, which we remove later.
% This is to avoid problems when we resample the signal since the resample
% function assumes the signal stops at zero it will force the last values
% to move towards zero.
xn = [x0; ones(addToEnd,1)*x0(end)];
yn = [y0; ones(addToEnd,1)*y0(end)];

%match the xn and yn with a time vector
timeStampn = [timeStamp, timeStamp(end)+Ts : Ts: timeStamp(end)+(addToEnd)*Ts];

%% resample the signals to 10 samples per second
[x, ~] = resample(xn, timeStampn, 1/Ts);
[y, ~] = resample(yn, timeStampn, 1/Ts);

% Remove the dummy values
x = x(1:end-addToEnd) + q(1,1); 
y = y(1:end-addToEnd) + q(1,2);

%Get a new time without the dummy values
newTime = 0:Ts:(length(x)-1)*Ts;

%Compute the heading
for i = 1:length(x)-1
    heading (i,1) = atan2( (y(i+1)-y(i)), (x(i+1)-x(i)) ); %theta
end

heading= [heading; heading(end)];
heading(1:2) = heading(3);

%collect everything in a struct
TrajSol.pos = [x, y, heading];
TrajSol.vel = refVel; 
TrajSol.timeSamp = newTime;











%% Plot the results

figure(322)
clf
subplot(3,1,1)
hold on
plot(timeStamp,q(:,1))
plot(newTime, x)
hold off

subplot(3,1,2)
hold on
plot(timeStamp,q(:,2))
plot(newTime, y)
hold off

subplot(3,1,3)
hold on
plot(timeStamp,refVel)
plot(newTime, v)
hold off
refVel = v;
heading=[];

figure(323)
clf
hold on
show(map)
plot(xway,yway)
plot(q(:,1), q(:,2))
hold off

figure(324)
clf
hold on
scatter(x,y)
hold off

figure(325)
clf
hold on
scatter(q(:,1), q(:,2))
hold off
q=[x' ; y'];
end