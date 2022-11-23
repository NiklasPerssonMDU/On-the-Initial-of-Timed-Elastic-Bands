function [ThetastarPathFound, ThetastarPath] = planThetastar (maze)
    map = maze.inflatedMap;
    gridSize = map.GridSize;
    startPos = maze.start;
    goalPos = maze.goal;
    [~, waypoints, plannedPath, ~, ~, ~, ~] =  thetaStar(map, startPos, goalPos, gridSize);
    if isempty(plannedPath)
        %this should never happen since A* has found a path, but better
        %safe than sorry
        ThetastarPathFound = 0;
        disp('Theta* failed to found a path, this is strange!')
        ThetastarPath = [];
    else
        ThetastarPathFound = 1;
%         if length(waypoints) == 3
%             middelWay = waypoints(2,1) - 
%         end
        %compute heading
            deltas = waypoints(2:end,:) - waypoints(1:end-1,:);
           
            ThetastarTheta = atan2(deltas(:,2), deltas(:,1));
            ThetastarTheta = [ThetastarTheta; ThetastarTheta(end)];

            %collect everything an offset to grid
            ThetastarPath = [waypoints-0.5, ThetastarTheta];
    end

end
