function [wayPoints, d] = getWaypoints(startPos, goalPos, parents)
    %get waypoints by starting from the goal point and traverse the parent
    %of that point
    %
        wayPoints = goalPos;
        d = 0;
        current = goalPos; %start at goal point
        while (current(1) ~= startPos(1)) || (current(2) ~= startPos(2)) 
            d = d + costFun(current, parents{current(1), current(2)});
            current = parents{current(1), current(2)};
            wayPoints = [current; wayPoints];
        end
    end