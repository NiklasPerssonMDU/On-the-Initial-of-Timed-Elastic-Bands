function [bikepath, startPos, SimTime] = getRefPos (refPos,Ts, yOff)
%extract x, y, heading
refX = refPos(:,1);
refY = refPos(:,2);
refTheta = -refPos(:,3);

%get start and goal pos
startPos = [refX(1), refY(1)]; 

%Get the simulation time
SimTime = length(refX) * Ts;


%Spline which is used to highlight the reference trajectory in simulation
refSpline = [ (refX - startPos(1)), (-refY + startPos(2) + yOff)];
refSpline = refSpline * [cos(refTheta(1)), -sin(refTheta(1)); sin(refTheta(1)), cos(refTheta(1))];

%Remove the first theta from the rest, so we always start at the same
%configuration with the bicycle.
refTheta = refTheta - (refTheta(1));
for i = 1:length(refTheta)
    refTheta(i) = atan2(sin(refTheta(i)), cos(refTheta(i)));
end

bikepath = [refSpline(:,1), -refSpline(:,2) , refTheta];
end