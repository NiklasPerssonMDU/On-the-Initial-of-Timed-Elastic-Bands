function [rect] = getObsCoords(occMatrix)

[rows, ~, ~] = size(occMatrix);


%Find out the max number of regions so we can initlaise a matrix for the
%columns among other things..
numRegions = zeros(rows,1);
for row = 1 : rows
    thisRow = occMatrix(row,:);
    [~, numRegions(row)] = bwlabel(thisRow);
end

maxNrRegions = max(numRegions);
nrOfCols = maxNrRegions * 2;

%initalise
cols = zeros(rows, nrOfCols);
lastCol = zeros (1, nrOfCols);
heights = repmat([0 1], rows, nrOfCols/2);




%go through the image and get all the start and end columns for each label
for row = 1 : rows
    thisRow = occMatrix(row,:);
    [regions, nR] = bwlabel(thisRow);
    for regionID = 1 : nR
        col1 = find(regions == regionID, 1, 'first');
        if ~isempty(col1)  %just making sure
            col2 = find(regions == regionID, 1, 'last');
        end
        cols(row, regionID*2-1:regionID*2) = [col1 col2]; %save the index of the columns
    end
end

%find out if the columns in the current row was also present in the last
%row, if so we add to the height
for row = 1 :rows
    currentCols = cols(row,:);
    for i = 2: 2: nrOfCols
        cc = currentCols (1, i-1:i);
        if isequal(cc, [0 0]) %[0 0] happens if nrOfRegions in the row is less than the maximum number of regions
            heights(row,i) = 0;
        else
            for j = 2: 2: length(lastCol)
                if isequal (cc, lastCol(1,j-1:j))
                    heights(row,i) = heights(row-1,j)+1; %get the last height and increase it by one
                    heights(row-1,j) = 0; %set the last height to zero, makes it easier to find the correct indecies later on
                    
                end
            end

        end
    end
    lastCol = currentCols;
end

%find all heights > 0 and retrive their row & columns
linIdx = find(heights > 0);
[r,c] = ind2sub (size(heights), linIdx);
rect = zeros(length(r),4);
rectPlot = zeros(length(r),4);

%Finally, we extract the rectangles positions
for i = 1 :length(r)
    rowStart = r(i) - heights(r(i),c(i)) +1;
    rowEnd = r(i);
    colStart = cols(r(i),c(i)-1);
    colEnd = cols( r(i),c(i));
    rect(i,:) = [colStart, colEnd, rowStart,rowEnd]; 
    rectPlot (i,:) = [colStart, rowStart, colEnd-colStart, rowEnd-rowStart+1]; %just for plotting
end
end