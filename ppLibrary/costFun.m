function cost = costFun (sParent, sprim)



if sParent(1) == sprim (1) || sParent(2) == sprim (2)
    dist = 1; %This is horisontal and vertical cost
else
    dist=1.4; %This is diagonal cost
end

cost = dist; 
end
