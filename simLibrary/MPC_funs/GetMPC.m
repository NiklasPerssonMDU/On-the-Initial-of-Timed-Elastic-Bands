function [BikeMPC] = GetMPC(BikeModel,v)
mpcverbosity off;
%% create MPC controller object with sample time
BikeMPC = mpc(BikeModel);
%% specify prediction horizon
BikeMPC.PredictionHorizon = 15; 
%% specify control horizon
BikeMPC.ControlHorizon = 3;  
%% specify nominal values for inputs and outputs
BikeMPC.Model.Nominal.U = [v;0];
BikeMPC.Model.Nominal.Y = [0;0;0;0;0];
%% specify scale factors for inputs and outputs
BikeMPC.MV(1).ScaleFactor = 1; %v in
BikeMPC.MV(2).ScaleFactor = 1.5; %phi ref in
BikeMPC.OV(1).ScaleFactor = 1; %yaw 
BikeMPC.OV(2).ScaleFactor = 1; %x
BikeMPC.OV(3).ScaleFactor = 2.5; %y 
BikeMPC.OV(4).ScaleFactor = 1; %phi
BikeMPC.OV(5).ScaleFactor = 1; %delta
%% specify constraints for MV and MV Rate
BikeMPC.MV(1).Min = v*0.5; 
BikeMPC.MV(1).Max = v*3; 
BikeMPC.MV(1).RateMin = -0.2;
BikeMPC.MV(1).RateMax = 0.2;
BikeMPC.MV(2).Min = -pi/6;
BikeMPC.MV(2).Max = pi/6;
BikeMPC.MV(2).RateMin = -deg2rad(5); 
BikeMPC.MV(2).RateMax = deg2rad(5);
%% specify constraint softening for MV and MV Rate
BikeMPC.MV(1).RateMinECR = 0.1;
BikeMPC.MV(1).RateMaxECR = 0.1;
BikeMPC.MV(2).RateMinECR = 0.1;
BikeMPC.MV(2).RateMaxECR = 0.1;
%% specify constraints for OV
BikeMPC.OV(1).Min = -pi; %yaw 
BikeMPC.OV(1).Max = pi; 
BikeMPC.OV(2).Min = -50; %x
BikeMPC.OV(2).Max = 50; 
BikeMPC.OV(3).Min = -50; %y
BikeMPC.OV(3).Max = 50; 
BikeMPC.OV(4).Min = -pi/4; %phi (lean)
BikeMPC.OV(4).Max = pi/4; 
BikeMPC.OV(5).Min = -pi/3; %delta (steer)
BikeMPC.OV(5).Max = pi/3;
%% specify weights
BikeMPC.Weights.MV = [0 0];
BikeMPC.Weights.MVRate = [0.1 0.1]; 
BikeMPC.Weights.OV = [5 10 50 20 0];
BikeMPC.Weights.ECR = 100000;
setEstimator(BikeMPC,'default');
end