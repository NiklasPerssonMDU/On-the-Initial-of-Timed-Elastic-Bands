
clc
clear
close all
addpath('C:\Users\npn01\OneDrive - Mälardalens högskola\Desktop\Research\BPP\model3D')
addpath('C:\Users\npn01\OneDrive - Mälardalens högskola\Desktop\Research\BPP\PP_simscapeModel\worldFunctions')
BicycleAssembly_DataFile;

Simulink.sdi.clear %Clear simulink data inspector

%% Create the world
load_system("BicycleModel_MPC");
%sub model where we build the world
submdl = 'BicycleModel_MPC/Subsystem/WorldSetup/';

load('refPath.mat');
%map name


%Sample rates
Ts=0.1; %Outer loop (10Hz)
TsInner=0.01; %Inner loop  (100Hz)

%If we use a random map, set the RndWorld = true (this takes longer time in
%a structured enviornment, but is faster for a random environement)
RndWorld = false;
newMap = false;
rotAng = 0;
setupWorld(submdl, originalMap, RndWorld, newMap, startPos, rotAng)


%% Simulation parameters
v=12; %nominal velocity in km/h

%Reference path
%refPathName=sprintf('bikepath%d',v);



%% Testing!! 

SFFLean = SFFLean*0.25;
%SFFLean = SFFLean*0;



%%

% 
% %PID parameters
% pidParam.Kp=-15;
% pidParam.Ki=-6;
% pidParam.Kd=-2;
% pidParam.N=240;

%PID parameters
% pidParam.Kp=-20;
% pidParam.Ki=-6;
% pidParam.Kd=-10;
% pidParam.N=100;

Ptest = 1.8;
Itest = 0.1;
Dtest = 0.001;
Ftest = 5;

k=33.9;
Tsm = 1/600; %600Hz
steer_sys = tf(k^2,[1,2*0.6*k,k^2],'InputDelay',0.015);
steerDyn=c2d(steer_sys,TsInner,'zoh');



pidParam.Kp=-15;
pidParam.Ki=-6;
pidParam.Kd=-3;
pidParam.N=240;

%Noise
rng('shuffle');
sigma_deltaDot=0; %disturbance in steering - variance (rad/s)
DeltaDotSeed=randi(1000);

%sigma_phi = 0;
sigma_phi = 0.001; %noise variance for lean angle (deg)
PhiSeed=randi(1000); 


tStart=0; %Time to start the trajectory tracking
SimTime=23; %simulation time in seconds


simParams= MPC_bicycle (refPoses, offset, v, pidParam, Ts, TsInner);


xoffset = 0;%(smiData.RigidTransform(49).translation(1)+smiData.RigidTransform(9).translation(1))/1000 + 0.0380; 
yoffset = 0;%smiData.RigidTransform(49).translation(2)/1000;
set_param('BicycleModel_MPC','SimMechanicsOpenEditorOnUpdate','on')
set_param('BicycleModel_MPC','StopTime',num2str(SimTime));
out=sim("BicycleModel_MPC.slx");


%% Present the results

%Plot position on the plane
figure(1)
clf
hold on
show(originalMap)
plot(out.x, out.y);
plot(simParams.refPath(:,2), simParams.refPath(:,3))
title('Position')
xlabel('x [m]')
ylabel('y [m]')

hold off

%%
%Plot lean angle
figure(2)
plot(out.LeanAngle.time, out.LeanAngle.signals.values)
title('Lean angle')
xlabel('Time[s]')
ylabel('Lean angle [deg]')


%%
%Plot heading for error handling
t=out.yaw.time;
tRef = (tStart:Ts:(simParams.EndOfPath*Ts)+tStart);
figure(3)
hold on
plot(t,rad2deg(out.yaw.signals.values))
plot(tRef, rad2deg(simParams.refPath(1:(end-10),1)))
title('Yaw angle')
xlabel('Time[s]')
ylabel('Yaw angle [deg]')
hold off

figure(4)
hold on
plot(tRef,simParams.refPath(1:(end-10),2))
plot(t,out.x)
title('x pos')
hold off

figure(5)
hold on
plot(tRef,simParams.refPath(1:(end-10),3))
plot(t,out.y)
title('y pos')
hold off



