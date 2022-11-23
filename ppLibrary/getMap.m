function [maze] = getMap (xlen, ylen, nObs, obsRadius)
%Create a randomised maze with size equal to xlen and ylen. The
%obstacles are inflated by the obsRadius parameter. nObs sets the
%number of obstacles in the map.
%Two copies are created,
% inflatedMap = the obstacles are inflated by the obsRadius
% originalMap = the obstacles are not inflated
%
% The inflated map is used for grid finding algorithms while the TEB
% uses the orignalMap. Thus, the grid finding algorihtms will have a
% safety distance of obsRadius, and the TEB obstacle safety distance is
% set in options to the TEB, these two should be the same.


%Keep track of the seed number to be able to reproduce the maps
%seedNr = randi(10000000);
    seedNr = 5106216;
    rng(seedNr)
rng(seedNr)
inflatedMap = mapMaze(8,3,'MapSize',[xlen ylen],'MapResolution',1);
rng(seedNr)
originalMap = mapMaze(8,3,'MapSize',[xlen ylen],'MapResolution',1);


%% Random obstacles
if nObs > 0.5
    %make sure it is an integer
    nObs = round (nObs);

    %get the free space
    occMat = flip(getOccupancy(inflatedMap));
    linIdx=find (occMat == 0);
    [freeC, freeR] = ind2sub (size(occMat),linIdx);
    freePos = [freeR, freeC];

    %randomise position of the obstacles in free space and place them
    %on the maps
    freePosIdx = randi([1 length(freePos)],1,nObs);
    setOccupancy(inflatedMap, freePos(freePosIdx,:), ones(length(freePosIdx),1));
    setOccupancy(originalMap, freePos(freePosIdx,:), ones(length(freePosIdx),1));
end

%Inflate the obstacles and the maze with obsRadius
inflate(inflatedMap, obsRadius);

%% Create random goal and start positon on the map
%Get free space on the map
occMat = flip(getOccupancy(inflatedMap));
linIdx=find (occMat == 0);
[freeC, freeR] = ind2sub (size(occMat),linIdx);
freePos = [freeR, freeC];

%For the first iteration, this is a obstacle for sure
goalPos = [0 0];
startPos = [0 0];

minSepD = euclidanDistance([1,1],[xlen,ylen]) / 2; % minimum distance between goal and start pos %<-- this was / 2

%Make sure we seperate the start and goal position
while euclidanDistance(startPos,goalPos) < minSepD
    sIdx = randi(length(freeC));
    gIdx = randi(length(freeC));

    startPos = freePos (sIdx,:);
    goalPos = freePos (gIdx,:);

end
maze.inflatedMap = inflatedMap;
maze.originalMap = originalMap;
maze.start = startPos;
maze.goal = goalPos;
maze.seedNr = seedNr;
end




