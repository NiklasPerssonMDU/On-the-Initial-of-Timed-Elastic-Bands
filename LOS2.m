function visible = LOS2(sParent, sprim, map)
        x0 = sParent(1);
        y0 = sParent(2);

        x1 = sprim(1);
        y1 = sprim(2);
        xoff=0;
        yoff=0;

%For debugging

res = 4; %this makes takes smaller step in the x and y direction, which
%leaves us a better chances of detecting an obstacle

        dy = y1 - y0; % distance in y direction between the nodes
        dx = x1 - x0; % distance in x direction between the nodes

        f = 0; % initailise f to 0
        
        % determine the direction between sparent and sprim y coords
        if dy < 0
            dy = -dy;
            sy = -1/res; % negative direciton
        else
            sy = 1/res; % positive direction
        end
        
        %determine the direction between sparent and sprim x coords
        if dx < 0
            dx = -dx;
            sx = -1/res; % negative direciton
        else
            sx = 1/res; % positive direction
        end
        
        %First we need to check if the distance is greater in the x or in
        %the y direction, so we know in which direction we should move
        %more. 
        if dx > dy


        %More or equal movement in the x direction compared to the y direction
            while x0 ~= x1 %if the two x coordinates are the same, the nodes are in LOS
                f = f + dy; %f is used to keep track if we should take a step in the x or y direction
                if f >= dx %We only take a step in the y dir if this is true
                    %check the occupancy grid map, if we detect an obstacle
                    %we return false and end.
                    if checkOccupancy(map, [x0+xoff + ((sx - 1) / 2), y0+yoff + ((sy - 1) / 2)])
                        visible = false;
                        return
                    end
                    %If no obstacle is detected, we move in the y direction
                    y0 = y0 + sy;
                    f = f - dx; %update the f value
                end
                if f ~= 0 && ... %check the current position
                        checkOccupancy(map, [x0+xoff + ((sx - 1) / 2), y0+yoff + ((sy - 1) / 2)])
                    visible = false;
                    return
                end
                if dy==0 && checkOccupancy(map, [x0+xoff + ((sx - 1) / 2), y0+yoff]) && checkOccupancy(map,[x0+xoff + ((sx - 1) / 2), y0+yoff - 1])  
                    visible = false;
                    return
                end
                x0 = x0 + sx; %In every iteration we move in the x dir
            end
        else %Less movement in the x direction compared to the y direction
            while y0 ~= y1
                f = f + dx; %f is used to keep track if we should take a step in the x or y direction
                
                if f >= dy
                    if checkOccupancy(map, [x0+xoff + ((sx - 1) / 2), y0+yoff+ ((sy - 1) / 2) ])
                        visible = false;
                        return
                    end
                    x0 = x0 + sx;
                    f = f - dy;
                end
                if f ~= 0 && ...
                        checkOccupancy(map, [x0+xoff + ((sx - 1) / 2), y0+yoff + ((sy - 1) / 2) ])
                    visible = false;
                    return
                end
             
                if dx==0 && checkOccupancy(map, [x0+xoff, y0+yoff + ((sy - 1) / 2)])  && checkOccupancy(map, [x0+xoff - 1, y0+yoff + ((sy - 1) / 2)]) 
                    visible = false;
                    return
                end
                y0 = y0 + sy;
            end
        end
        visible = true;
    end