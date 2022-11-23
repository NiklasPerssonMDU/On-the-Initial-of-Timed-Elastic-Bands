function [totalPath, d, waypoints, idxWayPoint] = retrivePath(startPos, goalPos, parents, map)
        totalPath = [];
        d = 0;
        waypoints = getWaypoints(startPos, goalPos, parents);
      
        for i = length(waypoints) : -1 : 2
            P1 = waypoints(i - 1,:);
            P2 = waypoints(i,:);
            [Px, Py] = bresenham(P1(1), P1(2), P2(1), P2(2));
            totalPath = [Px(2:end,1), Py(2:end,1); totalPath];
            ind(i)=length(totalPath); %save the indexes, used when computing the static flow field
            d = d + norm([P1(1) - P2(1), P1(2) - P2(2)]);
        end



        %%
        d = d * 1;
        %as we began from the goal position, we have to flip the indicies
        %of the waypoints.
        %first waypoint is the first index. 
        %the following indexes are obtained by flipping the obtained
        %indices
        %last index corresponds to the last node in the planned path
        idxWayPoint = [1, length(totalPath) - ind(3:end), length(totalPath)];
        
    end