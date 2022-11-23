function [offset, maze, trajSol] = planForBike (saveData)
xlen = 50; %x axis length
ylen = 50; %y axis length
NrOfObs = 0; %Number of obstacles
obsRadius = 1; %radius of obstacles

%% Get a map to plan on
maze = getMap (xlen, ylen, NrOfObs, obsRadius);

% size of the map
gridSize = maze.inflatedMap.GridSize;

%% Run thetaStar
[~, waypoints, ~, ~, openQ, closedQ, distance] = thetaStar(maze.inflatedMap, maze.start, maze.goal, gridSize);

if isempty(waypoints)
    disp('No path found')
    return
end

%% Smooth the path and find an optimal trajectory
offset = 0.5;
[trajSol] = optPath(waypoints-offset, distance, maze.originalMap);

%% Plot the results


figure(1)
clf
hold on
markerSz=80;
show(maze.originalMap)
%plot start and goal
scatter(maze.start(1)-offset, maze.start(2)-offset, markerSz, 'square', 'filled', 'g') %start
scatter(maze.goal(1)-offset, maze.goal(2)-offset, markerSz,'h','filled','r') %goal
%plot the path found by Theta*
plot(waypoints(:,1)-offset, waypoints(:,2)-offset,'linewidth',2)
%plot smooth path
plot(trajSol.pos(:,1), trajSol.pos(:,2), 'linewidth' ,2)
title ('Planned path using opt. Theta*')
legend ('Start pos', 'Goal pos', 'Theta* path', 'Smooth path', 'Location','northeastoutside')
axis ([0, xlen, 0, ylen])
hold off


%% get open queue and closed queue indicies
% [closedQx, closedQy] = ind2sub(size(closedQ),find(closedQ));
% [openQx, openQy] = ind2sub(size(openQ),find(openQ));
%% plot open and closed queue
% figure(4)
% hold on
% show(maze.originalMap)
% scatter(closedQx-1, closedQy-1, 'square', 'filled', 'y') %closed queue
% scatter(openQx-1, openQy-1, 'square', 'filled', 'b') %open queue
% hold off

if saveData
    save('refPath.mat','offset', 'maze', 'trajSol');
end



