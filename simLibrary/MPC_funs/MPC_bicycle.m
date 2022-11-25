function [simParams ] = MPC_bicycle(refPoses, v, pidParam, Ts)
%% Bicycle model parameters
BikeParam.a      = 0.4728; % horisontal distance from the rear wheel to CoG
BikeParam.v      = v/3.6; % nominal velocity
BikeParam.h      = 0.5151; % height of the CoG from the ground plane
BikeParam.b      = 1.08; %wheel base
BikeParam.g      = 9.82; %gravity
BikeParam.lambda = deg2rad(72.95);  %caster angle
BikeParam.p      = sin(BikeParam.lambda);
BikeParam.c      = 0.087; %trail (m)
BikeParam.r      = 0.349; %wheel radius [m]


%% Compute discrete time bike model
BikeModel = GetBikeModel(pidParam, BikeParam, Ts);

%% Fix reference path for the bicycle
%rearrange x,y and yaw to match the states of the model
BikePath(:,1) = refPoses(:,3);
BikePath(:,2) = refPoses(:,1);
BikePath(:,3) = refPoses(:,2);

BikePath = [BikePath, zeros(length(BikePath),2)]; %Append 2 empty columns to reprsent reference lean pos and steer pos

%% Configure MPC
BikeMPC = GetMPC(BikeModel, BikeParam.v);


%So we stop the simulation before hitting the end of the path, thus
%avoiding errors
EndOfPath = length(BikePath)-BikeMPC.PredictionHorizon-1; 

%% Collect everything in a struct
simParams.bike = BikeParam;
simParams.mpc = BikeMPC;
simParams.EndOfPath = EndOfPath;
simParams.refPath = BikePath;

end

