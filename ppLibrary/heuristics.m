function [H] = heuristics(gridSize, goal, weight)

% eucledian distance as heuristics

H = zeros(gridSize);
for i = 1:gridSize(1) 
    for j = 1:gridSize(2)
        H(i,j) = weight * sqrt((i-goal(1))^2 + (j-goal(2))^2);
    end
end

end
