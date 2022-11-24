function [] = setupWorld(map, newMap, startPos)
%sub model where the world is created
submdl = 'BicycleModel_MPC/Bicycle/WorldSetup/';
offset = 0.5;

%if we have a new map, we clear the latest map
if newMap
    bl = getfullname(Simulink.findBlocks(submdl));
    nBlocks = length(bl);
    except1 = [submdl, 'Conn1'];
    except2 = [submdl, 'Bicycle'];
    for i = 1 : nBlocks
        if ~ (strcmp (bl{i}, except1) || strcmp(bl{i}, except2) )  %don't want to remove the output ports
            delete_block(bl{i})
        end
    end
    %Clear the lines
    delete_line(find_system(submdl, 'FindAll', 'on', 'Type', 'line', 'Connected', 'off'))
end




%Get the occupancy matrix
occMatrix = getOccupancy(map);

szMap = size(occMatrix);
bricks = getObsCoords(occMatrix);
for i=1:size(bricks,1)
    %create obstacles with unique transformations
    obsName = sprintf('NewBrickSolid%d',i);
    transformName = sprintf('Obs%dTransform',i);
    SCFname = sprintf('SCF%d',i);

    if newMap
        %add the solids and their transformations blocks to the model
        add_block('sm_lib/Body Elements/Brick Solid', [submdl,obsName] ...
            , 'Position', [350, i*100+50, 400, (i+1)*100] ... %[Left top right bottom]
            , 'Orientation','left');
        add_block('sm_lib/Frames and Transforms/Rigid Transform',[submdl, transformName] ...
            , 'Position', [250, i*100+50, 300, (i+1)*100]);

        %connect the solid with its corresponding transformation
        add_line(submdl,[transformName,'/RConn 1'],[obsName,'/RConn 1'])

        %connect the transformation with output port
        add_line(submdl,[transformName,'/LConn 1'], 'Conn1/RConn 1','autorouting','on')

        %enable export geometry (requried to model forces between
        %bicycle and solid)
        set_param ([submdl, obsName],'ExportEntireGeometry','On');

        %FOR TESTNG
        add_block('sm_lib/Forces and Torques/Spatial Contact Force', [submdl,SCFname] ...
            , 'Position', [450, i*100+50, 500, (i+1)*100]);

        %connect the export options of solids to the second output port
        add_line(submdl, [SCFname,'/LConn 1'], [obsName,'/LConn 1'],'autorouting','on')
        add_line(submdl, [SCFname,'/RConn 1'], 'Bicycle/RConn 1','autorouting','on')
    end
    %compute dimension of the block
    ydim = (bricks(i,2) - bricks(i,1) + 1); % 1+ so we don't end up with 0 width or height
    xdim = (bricks(i,4) - bricks(i,3) + 1);

    %set dimension of the block
    brickDim = ['[',num2str([xdim,ydim,1]),']'];
    set_param ([submdl, obsName], 'BrickDimensions', brickDim)

    %set color of the block
    set_param([submdl, obsName],'GraphicVisPropType','AdvancedVisualProperties',...
        'GraphicDiffuseColor','[0.6862745 0.8745098 0.654902]','GraphicAmbientColor', '[0.15 0.3 0.15 1.0]');


    [xOff, yOff] = firstFrameOffset;
    %we have to rotate the x, so we start at the other end of the map
    xpos = -szMap(1) + startPos(2) + (bricks(i,3) + (bricks(i,4) - bricks(i,3)) / 2) - xOff +offset;
    ypos = startPos(1) - (bricks(i,1) + (bricks(i,2) - bricks(i,1)) / 2 + yOff -offset );

    set_param([submdl,transformName],'TranslationMethod','Cartesian'...
        , 'TranslationCartesianOffset', ['[',num2str([xpos, ypos, 0]),']']);

end

end