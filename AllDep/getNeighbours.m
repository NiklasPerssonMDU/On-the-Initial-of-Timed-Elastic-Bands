function [Neighbours]=getNeighbours (gridSize, map, s)
occMatrix=flip(getOccupancy(map)); 
Neighbours = [];
% Search 9 nodes
for i = s(1)-1:s(1)+1
        for j = s(2)-1:s(2)+1
            % First we test so the neighbour is valid, three requriments 
            % need to be fulfilled:
            % 1. The neighbour should not be our current position
            % 2. The neighbour should be within the map
            % 3. The neighbour should not be an obstacle


            if i == s(1) && j == s(2) %Req 1.
                % We are at the current node s, lets ignore it and jump to
                % next iteration of the loop
                continue 
            elseif i < 1 || j < 1 || i > gridSize(1) || j > gridSize(2) %Req 2.
                % The node is out of bounds
                continue
            elseif checkOccupancy(map,[i, j]) %Req 3.
                     continue
            end
         

            % Now we are convinced we have found a valid neighbour that
            % fulfills the three requriments
            Neighbours = [Neighbours; i, j];

        end
end


end