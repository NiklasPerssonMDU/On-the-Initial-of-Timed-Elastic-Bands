function [trajSol] = TEB (nonOptPath, map)
v = 12; 
d = pathDistance(nonOptPath(:,1), nonOptPath(:,2));

%estimate number of samples, sampling time of 0.1s
NumSamples = round((d / (v/3.6)) * 10);

%% set options
options = optimizePathOptions;
options.MinTurningRadius = 3; % meters
options.MaxVelocity = 5; % m/s
options.MaxAcceleration = 2; % m/s/s
options.ReferenceDeltaTime = 0.1; % second
options.MaxPathStates = NumSamples; 
options.ObstacleSafetyMargin = 1; %m
options.WeightSmoothness = 1000;
options.WeightVelocity = 1;
options.WeightTime = 20;
options.WeightMinTurningRadius = 10;
options.WeightObstacles = 5;
options.WeightAcceleration = 1;
options.WeightAngularVelocity = 1;
options.WeightAngularAcceleration = 1;


%% optimize the trajectory
[optpath, kinInfo] = optimizePath(nonOptPath ,map, options);

refVel = [kinInfo.Velocity; 0]'; %, kinInfo
timeSamp = kinInfo.TimeStamps';

%collect everything in a struct
trajSol.pos = optpath; 
trajSol.vel = refVel; 
trajSol.timeSamp = timeSamp;

end
