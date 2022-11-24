clc
clear
close all


addpath(genpath('simLibrary'))
addpath ppLibrary\
%run this file to intialise the simscape model
BicycleModel_DataFile;



%load the model
load_system("BicycleModel_MPC");
%chose if you want to save the data into csv files or not
saveData = 0;

%% Plan a path using Theta* and optimise using Time elastic bands
[offset, maze, trajSol] = planForBike(saveData);

%% Sample rates
Ts=0.1; %Outer loop (10Hz)
TsInner=0.01; %Inner loop  (100Hz)

%% Balance control
pidParam.Kp = 15;
pidParam.Ki = 10;
pidParam.Kd = 4;
pidParam.N = 200;

%% nominal forward velocity
v=10;

%% compute offset in the first frame
[xOff, yOff] = firstFrameOffset;

%% Get the reference positions
[bikepath, startPos, SimTime] = getRefPos (trajSol.pos, Ts, yOff);

%% Setup the world
newMap = true; %we remove the last map and create a new
setupWorld(maze.originalMap, newMap, startPos)

%% get the MPC
simParams = MPC_bicycle (bikepath, v, pidParam, Ts);

%% set parameters for simulation
set_param('BicycleModel_MPC','SimMechanicsOpenEditorOnUpdate','on',...
        'StopTime',num2str(SimTime));

%% run simulation
out=sim("BicycleModel_MPC.slx");

%% -----------Simulation is done----------- %% 
figure(2)
clf
show(maze.originalMap) 
hold on
% Plot the reference path
plot(trajSol.pos(1:simParams.EndOfPath,1), trajSol.pos(1:simParams.EndOfPath,2),'linewidth',2);

%rotate back the path travelled by the bike and plot it
angToRot = -trajSol.pos(1,3);
outPath = [out.x, out.y];
outPath = outPath * [cos(angToRot), -sin(angToRot); sin(angToRot), cos(angToRot)];
outPath = [(outPath(:,1) + startPos(1)), (outPath(:,2) + startPos(2) - yOff)];
plot(outPath(:,1), outPath(:,2), 'linewidth',2)
legend ('Reference path', 'Bicycle path','Location','northeastoutside')
title('Path planned with Optimised Theta*')
subtitle('Tracked by autonomous bicycle')
xlabel('x [m]')
ylabel('y [m]')
hold off


%% Save the data
if saveData
    xSave = out.x;
    ySave = out.y;

    refTheta = -trajSol.pos(1,3);

    bikePath = [out.x, out.y] * [cos(refTheta), -sin(refTheta); sin(refTheta), cos(refTheta)];
    bikePath = [ (bikePath (:,1) + startPos(1) - offset), (bikePath (:,2) + startPos(2) - offset - yOff)];

    mpcData = table(bikePath (:,1)+0.5, bikePath (:,2)+0.5, trajSol.pos(1:length(out.x),1), trajSol.pos(1:length(out.x),2));

    mpcData.Properties.VariableNames(1:4) = {'x','y','refx','refy'};
    writetable(mpcData,'MPCdata.csv');

    occMatrix = flip(getOccupancy(maze.originalMap));
    k=1;
    for i = 0:size(occMatrix,1)-1
        for j = 0:size(occMatrix,2)-1

            LatTable(k,:) = [j, i, occMatrix(i+1,j+1)];
            k = k +1;
        end
    end

    MPCmap=table(LatTable(:,1)+0.5, LatTable(:,2)+0.5, LatTable(:,3));
    MPCmap.Properties.VariableNames(1:3) = {'x','y','ObsValue'};
    writetable(MPCmap,'MPCmap.csv');
end
