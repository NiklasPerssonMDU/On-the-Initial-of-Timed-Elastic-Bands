clc
clear
close all
addpath(genpath('simLibrary'))

%run this file to intialise the simscape model
BicycleModel_DataFile;
%load the model
load_system("BicycleModel");

%% noise parameters
sigmaSteer = 0.701;
sigmaLean = 10^(-3/2);
rndSeed = randi(10000); %make sure we don't use the same random numbers

%% Sample rates
Ts=0.1; %Outer loop (10Hz)
TsInner=0.01; %Inner loop  (100Hz)

%% Balance  PID control
pidParam.Kp = 15;
pidParam.Ki = 10;
pidParam.Kd = 4;
pidParam.N = 200;

%% nominal forward velocity
v=14; %km/h

%% simulation time
SimTime=5;


%% set parameters for simulation
set_param('BicycleModel','SimMechanicsOpenEditorOnUpdate','on',...
        'StopTime',num2str(SimTime));

%% run simulation
out=sim("BicycleModel.slx");

%% -----------Simulation is done, plot results----------- %% 
figure(1)
subplot(2,1,1)
plot(out.x, out.y, 'linewidth',2)
title('Bicycle path')
xlabel('x [m]')
ylabel('y [m]')
hold off

LeanTime = 0:TsInner:length(out.rollRad)*TsInner-TsInner;
leanDeg = rad2deg(out.rollRad);

subplot(2,1,2)
plot(LeanTime, leanDeg, 'linewidth',2)
title('Bicycle lean angle')
xlabel('Time [s]')
ylabel('Lean [deg]')
hold off