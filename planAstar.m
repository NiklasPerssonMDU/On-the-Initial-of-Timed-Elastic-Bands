function [AstarPathFound, AstarPath] = planAstar (maze)
        %Get parameters from the maze struct
        map = maze.inflatedMap;
        startPos = maze.start;
        goalPos = maze.goal;
        ylen = map.GridSize(2);

        %Compute start and goal for A*
        startP = [round(ylen-startPos(2)+1), startPos(1)];
        goalP = [round(ylen-goalPos(2)+1), goalPos(1)];

        %plan the path
        planner = plannerAStarGrid(map,'Hcost','Euclidean','DiagonalSearch','On');
        plannedPath = plan(planner,startP,goalP);

        if ~isempty(plannedPath)
            % flip the positions back so it is consistent with the map
            AstarX = plannedPath(:,2) - 0.5;
            AstarY = ylen-plannedPath(:,1) + 0.5;
            
            %compute the heading
            dy = AstarY(2:end) - AstarY(1:end-1);
            dx = AstarX(2:end) - AstarX(1:end-1);
            AstarTheta = atan2(dy,dx);
            AstarTheta = [AstarTheta; AstarTheta(end)];

            %Set the boolean to true since we found a path
            AstarPathFound = true;
            
        else
            %If A* doesn't find a path, we set the AstarPathFound flag to
            %false and return an empty path
            AstarPathFound = false;
            AstarX = [];
            AstarY = [];
            AstarTheta = [];
        end

        AstarPath = [AstarX, AstarY, AstarTheta];