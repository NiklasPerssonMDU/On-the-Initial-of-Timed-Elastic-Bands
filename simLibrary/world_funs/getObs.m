function [brick] = getObs(occMatrix)
    
    %Check connectctivty in the rows
    rowConnected = bwconncomp (occMatrix, [0 0 0; 1 1 1; 0 0 0]);

    %Check connectctivity in the columns
    colConnected = bwconncomp (occMatrix, [0 1 0; 0 1 0; 0 1 0]);

    %bwconncomp (occMatrix, )
    %initalise a counter
    k = 1;
    usedLinInd = [];
    


    for i = 1 : length(colConnected.PixelIdxList)
        %This loop is only for cells connected columnwise

        if length(colConnected.PixelIdxList{i}) > 1 %only extract cells with more than one element
            %convert to subscripts and compute the dimensions of the brick
            [row, col] = ind2sub( size(occMatrix), colConnected.PixelIdxList{i});
            brick(k,:) = [row(1), row(end), col(1), col(end)];

            %Keep track of which linear indicies have been used
            usedLinInd = [usedLinInd; colConnected.PixelIdxList{i}];
            k = k + 1;
        end
    end

    for i = 1 : length(rowConnected.PixelIdxList)
        %This loop is only for cells connected rowwise

        if length(rowConnected.PixelIdxList{i}) > 1
            %convert to subscripts and compute the dimensions of the brick
            [row, col] = ind2sub( size(occMatrix), rowConnected.PixelIdxList{i});
            brick(k,:) = [row(1), row(end), col(1), col(end)];

            %Keep track of which linear indicies have been used
            usedLinInd = [usedLinInd; rowConnected.PixelIdxList{i}];
            k = k + 1;
        end
    end

    %Deal with the singleCells, if any
    linInd = find(occMatrix);
    singleCells = setdiff(linInd, usedLinInd);
    if ~isempty(singleCells)
        [singleCellRow, singleCellCol] = ind2sub(size(occMatrix) , singleCells);
    end


end