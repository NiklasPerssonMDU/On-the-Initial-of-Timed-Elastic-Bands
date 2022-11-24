function [curv, cumArcLength, intPath] = compCurv (D, plannedPath)

Ts = 0.1; %Sampling time
v = 12/3.6; %velocity. used to interpolate the paths to compute the curvature

%compute the number of samples
nSamples = round( (D/v) / Ts);

%interpolate the path
intPath = interparc (nSamples, plannedPath(:,1), plannedPath(:,2), 'linear');

%compute the radius of the curvature and the cumulative arc length
[L,R,~] = curvature(intPath);

%compute the curvature
curv = sum(1./R(2:end-1));
cumArcLength =  L(end);
end