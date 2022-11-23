
function [parents, waypoints, plannedPath, idxWaypoints, openQueue, closedQueue, distance] = thetaStar (map, startPos, goalPos, gridSize)
%% Theta* 
waypoints = [];
plannedPath = [];
idxWaypoints = [];
distance = [];


% Intialise the cost matrix g
g = inf(gridSize);

% start pos has a cost of 0
g(startPos(1),startPos(2)) = 0;

%f score is defined as g+h for each node
f = zeros(gridSize);

% Open and closed queues are boolean matrices with size equal to mapsize
% Both initialsed to false for all nodes
% If a node is in the closed or open queue, the corresponding position is changed to true
closedQueue = false(gridSize);
openQueue = false(gridSize);

% Input start pos to open queue
openQueue(startPos(1),startPos(2)) = true;

%A cell matrix that keeps track of the parent of each cell
%i.e the parent of the cell 4,4 can be found in position 4,4 in the matrix
parents = cell (gridSize(1), gridSize(2));

% parent of the start node is the start node itself
parents(startPos(1),startPos(2)) = {startPos};

% compute heuristics for all points on the map to the goal point
% the weight can be used to penatilize the diagonals more.
weight = 1;
h = heuristics(gridSize, goalPos, weight);


% insert the start node into the open queue
%openQueue = insertNode(startPos, g(startPos(1),startPos(2)), h(startPos(1),startPos(2)), parent,[]);
s=zeros(1,2);
% Main algorithm start
while any(any(openQueue)) % Continue as long as there is nodes in the open queue

    % s is the most promising node from the open queue, i.e lowest fscore
    [~, idx] = min(f(openQueue));
    linearidx = find(openQueue,idx); %extract the linear index
    [s(1),s(2)] = ind2sub(gridSize, linearidx(end)); %convert from linear index to a subscript
    
    %Remove the current node s from the open queue
    openQueue(s(1),s(2)) = false;

    % if we are at the goal pos we are done and can retrive the path by
    % stepping back through the parents
    if s == goalPos
        %disp('Path found, retrive by stepping back through the parents')
        [plannedPath,distance, waypoints, idxWaypoints] = retrivePath(startPos, goalPos, parents, map);
        break
    end
    
    %Move node s to the closed queue
    closedQueue(s(1), s(2)) = true;


    neighbours=getNeighbours(gridSize, map, s);
    if isempty(neighbours) %if we don't find any neighbours we can't compute a path, so lets break and go outside and take some fresh air
        break
    end

    for i = 1:size(neighbours,1)
        sPrim = neighbours(i,:); 
            if ~closedQueue(sPrim(1),sPrim(2))
                
                [parents, g, openQueue, f]=updateVertex(s, sPrim, parents, g, h, f, openQueue, map);
                
            end
    end

end

end



