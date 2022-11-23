function [] = randomWorld(occMatrix, submdl, NewMap, startPos, res)
%get the size of the map
szMap = size(occMatrix);

%Find the obstacles
linInd = find(occMatrix);

%Find the row and column indicies of the obsatcles
[obsX,obsY] = ind2sub(size(occMatrix),linInd);

%Loop through all obstacles and place them in the enviornment
nrOfObs = length(linInd);



if nrOfObs == 0
    %we don't change anything if we don't have any obstacles
    return
end



for i=1:nrOfObs
    %create obstacles with unique transformations
    obsName = sprintf('NewBrickSolid%d',i);
    transformName = sprintf('Obs%dTransform',i);
    SCFname = sprintf('SCF%d',i);
    if NewMap
        %add the solids and their transformations blocks to the model
        add_block('sm_lib/Body Elements/Brick Solid', [submdl,obsName] ...
            , 'Position', [350, i*100+50, 400, (i+1)*100] ... %[Left top right bottom]
            , 'Orientation','left');
        add_block('sm_lib/Frames and Transforms/Rigid Transform',[submdl, transformName] ...
            , 'Position', [250, i*100+50, 300, (i+1)*100]);

        %connect the solid with its corresponding transformation
        add_line(submdl,[transformName,'/RConn 1'],[obsName,'/RConn 1'])
        add_line(submdl,[transformName,'/LConn 1'], 'Conn1/RConn 1','autorouting','on')

        %enable export geometry (requried to model forces between
        %bicycle and solid)
        set_param ([submdl, obsName],'ExportEntireGeometry','On');

        add_block('sm_lib/Forces and Torques/Spatial Contact Force', [submdl,SCFname] ...
             , 'Position', [450, i*100+50, 500, (i+1)*100]);

        %connect the export options of solids to the second output port
            add_line(submdl, [SCFname,'/LConn 1'], [obsName,'/LConn 1'],'autorouting','on')
            add_line(submdl, [SCFname,'/RConn 1'], 'Bicycle/RConn 1','autorouting','on')

        %add_line(submdl, [obsName,'/LConn 1'], 'Conn2/RConn 1','autorouting','on')
    end
    %set position of the block in the world
    xpos = ((szMap(1)-obsX(i))/2)/res-startPos(2);
    ypos = (obsY(i)/2)/res-startPos(1);
    set_param([submdl,transformName],'TranslationMethod','Cartesian'...
        , 'TranslationCartesianOffset', ['[', num2str([xpos, ypos, 0]), ']']);
end

%  set_param([submdl,'Obs1Transform'],'TranslationMethod','Cartesian'...
%         , 'TranslationCartesianOffset', ['[', num2str([25-16, 2, 0]), ']']);

end
%         ydim = 1/res; % 1+ so we don't end up with 0 width or height
%         xdim = 1/res;
% 
%         %set dimension of the block
%         brickDim = ['[',num2str([xdim,ydim,1]),']'];
%         set_param ([submdl, obsName], 'BrickDimensions', brickDim)