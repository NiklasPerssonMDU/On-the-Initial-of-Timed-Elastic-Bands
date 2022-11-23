function [totD] = pathDistance(x,y)

totD = sum(sqrt((x(1:end-1)-x(2:end)).^2 + (y(1:end-1)-y(2:end)).^2));

if totD == 0 
    totD=NaN;
end

end