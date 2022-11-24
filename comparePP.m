clear
clc
close all
addpath ppLibrary\

%Initalise variable to hold the result of the path planners and information
%about the maze
PPsol = cell(5,4);
PPsol{1,1} = 'Planner';
PPsol{1,2} = 'OptPath';
PPsol{1,3} = 'NonOptPath';
PPsol{1,4} = 'MazeInfo';
PPsol{2,1} = 'A*';
PPsol{3,1} = 'Theta*';
PPsol{4,1} = 'Hybrid A*';
PPsol{5,1} = 'RRT*';

[AllPPsol(1:100,1:3).data] = deal(PPsol);

%% create 300 mazes of size 100 by 100 and obstacle inflation of 1m
xlen=100;
ylen=100;
obsRadius = 1;
maze = get300Maps(xlen, ylen, obsRadius);

%Construct a data queue to send messages
q = parallel.pool.DataQueue;
afterEach(q, @disp);

%% Plan paths on all maps using A*, Theta*, Hybrid A*, RRT*
for j = 1:3
    clc
    parfor i = 1:100
        rerun = true;
        warning ('off','all');

        while rerun
            %% Plan using A*
            % If we can't find a path using A* there is no reason to run the others
            [pathFoundAstar, AstarPath] = planAstar(maze(j,i));
            if ~pathFoundAstar
                send(q,['A* failed in iteration j=', num2str(j), ', i=', num2str(i)])
            else
                %% Plan using Theta*
                [~,ThetastarPath] = planThetastar(maze(j,i));
                %if we found a path using A*, we will find a path using Theta*
                %since both planners are complete.
                
                %Extract the first and last to be used in Hybrid A* and
                %RRT* as we know need the pose, and not just the position
                headings = [ThetastarPath(1,3), ThetastarPath(end,3)];

                %% Plan using Hybrid A*
                [pathFoundHybridAstar, HybridAstarPath] = planHybridAstar(maze(j,i), headings);
                %Hybrid A* might fail since it's not a complete path planner,
                %so we have to check again before running RRT*
                if ~pathFoundHybridAstar
                    send(q,['Hybrid A* failed in iteration j=', num2str(j), ', i=', num2str(i)])
                    
                else
                    %% Plan using RRT*
                    [pathFoundRRTstar, RRTstarPath] = planRRTstar(maze(j,i), headings);
                    %If RRT* fails we want to know it
                    if ~pathFoundRRTstar
                        send(q,['RRT* failed in iteration j=', num2str(j), ', i=', num2str(i)])
                        
                    end
                end
            end

            %% Store the information of the paths for future optimisation
            if pathFoundAstar && pathFoundHybridAstar && pathFoundRRTstar
                rerun = false;
                AllPPsol(i,j).data{2,2} = [0 0 0];%A* opt 
                AllPPsol(i,j).data{3,2} = [0 0 0];%Theta* opt 
                AllPPsol(i,j).data{4,2} = [0 0 0];%Hybrid A* opt 
                AllPPsol(i,j).data{5,2} = [0 0 0];%RRT* opt 

                AllPPsol(i,j).data{2,3} = AstarPath;%A* non-opt 
                AllPPsol(i,j).data{3,3} = ThetastarPath;%Theta* non-opt 
                AllPPsol(i,j).data{4,3} = HybridAstarPath;%Hybrid A* non-opt 
                AllPPsol(i,j).data{5,3} = RRTstarPath;%RRT* non-opt 
                
                AllPPsol(i,j).data{2,4} = maze(j,i);
                AllPPsol(i,j).data{3,4} = j;
                AllPPsol(i,j).data{4,4} = i;
                
                %% If one wants to plot the resulting paths
                %plotPP(AstarPath, ThetastarPath, HybridAstarPath, RRTstarPath, maze(j,i))
            else
                %If any path planner fails on this particular map, we get a new
                %map and rerun the iteration
                rerun = true;
                nObs = 50*(j-1);
                maze(j,i) = getMap (xlen, ylen, nObs, obsRadius);
            end
        end
    end
end

%% Optimise the paths using Time Elastic Bands
for j = 1:3
    parfor i = 1:100
        PPsol = AllPPsol(i,j).data;
        AllPPsol(i,j).data{2,2} = TEB (PPsol{2,3}, PPsol{2,4}.originalMap); 
        AllPPsol(i,j).data{3,2} = TEB (PPsol{3,3}, PPsol{2,4}.originalMap); 
        AllPPsol(i,j).data{4,2} = TEB (PPsol{4,3}, PPsol{2,4}.originalMap); 
        AllPPsol(i,j).data{5,2} = TEB (PPsol{5,3}, PPsol{2,4}.originalMap);
        send(q,['Iteration j=', num2str(j), ', i=', num2str(i), ' complete'])
    end
end
