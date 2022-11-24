
function [DiscBike] = GetBikeModel(pidParam, BikeParam, Ts)
a=BikeParam.a; 
v=BikeParam.v;
h=BikeParam.h;
b=BikeParam.b; 
g=BikeParam.g;
p=BikeParam.p;
c=BikeParam.c;

Kp=pidParam.Kp;
Ki=pidParam.Ki;
Kd=pidParam.Kd;
N=pidParam.N;

%% Open loop dynamics (PID -> steering-> roll) with state matrices
A= [-N                 0        0              0  0                           0;
    1                  0        0              0  0                           0;
    100*(Ki-Kd*N^2) 100*Ki*N   -100            0  0                           0;
    0                  0       -1*a*p*v/(b*h)  0  g/h   (p/(b*h))*((g*c*a*p/h)-v^2);
    0                  0        0              1  0                           0;
    0                  0        1              0  0                           0];
B=[1; 0; 100*(Kp+Kd*N); 0; 0; 0];
C=[0 0 0 0 1 0];
%%  Closed loop dynamics (feedback(PID -> Steering -> roll)) 
Acl=(A+B*C);
Bcl=B;

Ccl=[0 0 0 0 1 0;
     0 0 0 0 0 1];
Dcl=0;

CLss=ss(Acl,Bcl,Ccl,Dcl,'StateName',{'e1' 'e2','deltaDot','phiDot', 'phi', 'delta'},...
     'InputName','phi_ref','OutputName',{'phi','delta'});

%% Kinematics
%To compute the yaw angle, the steering angle is needed as an input, but in
%the complete system the steering angle is a state, so we modify the system
%below instead (BikeModel.A(1,9)=p*v/b)
Ak=[0 0 0;
    0 0 0; %x=v*cos(yaw) -> v is input and cos(yaw)=1
    v 0 0];%y=v*yaw -> v is consant v and yaw is just yaw for small angle approx with sin(yaw)
Bk=[0; 1; 0];
Ck=[1 0 0;
    0 1 0;
    0 0 1];
Dk=[0];
Kinss=ss(Ak,Bk,Ck,Dk,'StateName', {'Yaw', 'x', 'y'},'InputName','v','OutputName',{'Yaw','x','y'});


%% Complete system discretizied using zoh method
BikeModel=append(Kinss,CLss);
BikeModel.A(1,9)=p*v/b;
DiscBike=c2d(BikeModel,Ts,'zoh');


end

