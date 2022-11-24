function [RRTstarPathFound, PlannedPath] = planRRTstar(maze, headings)
%Extract relevant parameters
%For RRT* we need the pose, we get the headings from Theta*
map = maze.inflatedMap;
startPose = [maze.start-0.5, headings(1)];
goalPose = [maze.goal-0.5, headings(2)];

%set state space and validator
ssRRT = stateSpaceSE2;
ssRRT.StateBounds = [map.XWorldLimits;map.YWorldLimits;[-pi pi]];
svRRT = validatorOccupancyMap(ssRRT);
svRRT.Map = map;
svRRT.ValidationDistance = 0.1;

%configure the planner
RRTplanner = plannerRRTStar(ssRRT,svRRT, ...
    MaxIterations=100000, ...
    ContinueAfterGoalReached=true, ...
    MaxConnectionDistance=10, ...
    MaxNumTreeNodes=30000);

%plan a path
[RRTstarPath, solInfo] = plan(RRTplanner,startPose,goalPose);
PlannedPath = RRTstarPath.States;
RRTstarPathFound = solInfo.IsPathFound;
