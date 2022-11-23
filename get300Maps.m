function [maze] = get300Maps (xlen, ylen, obsRadius)
%Get 300 randomised maps with a size of xlen * ylen and inflate the
%obstacles by obsRadius

for j= 1:3
    for i = 1:100
        %maze 1   -> 100 - no  random obstacles
        %maze 101 -> 200 - 50  random obstacles
        %maze 201 -> 300 - 100 random obstacles
        nObs = 50*(j-1);
        maze(j,i) = getMap (xlen, ylen, nObs, obsRadius);
    end
end

end