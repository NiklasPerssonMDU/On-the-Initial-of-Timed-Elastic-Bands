function [HybridAstarPathFound, PlannedPath] = planHybridAstar (maze, headings)
%Extract relevant parameters
%For Hybrid A* we need the pose, we get the headings from Theta*
map = maze.inflatedMap;
startPose = [maze.start-0.5, headings(1)];
goalPose = [maze.goal-0.5, headings(2)];

%set state space and validator for Hybrid A*
ss = stateSpaceSE2;
ss.StateBounds = [map.XWorldLimits; map.YWorldLimits; [-pi pi]];
sv = validatorOccupancyMap(ss);
sv.Map = map;
sv.ValidationDistance = 0.1;

%Plan the path
%keep reverse cost and direction switchingcost high to avoid reverse
%motion.
planner = plannerHybridAStar(sv,'MinTurningRadius', 3 ...
    ,'MotionPrimitiveLength',1.5,'NumMotionPrimitives',15 ...
    ,'ReverseCost',100000,'DirectionSwitchingCost',100000);


[HybridAstarPath, directions, solInfo] = plan(planner,startPose,goalPose);

if isempty(find(directions == -1,1))
    PlannedPath = HybridAstarPath.States;
    HybridAstarPathFound = solInfo.IsPathFound;
else
    %we might have found a path, but we are using reverse motion so we
    %discard the path
    PlannedPath = HybridAstarPath.States;
    HybridAstarPathFound = false;
end


end